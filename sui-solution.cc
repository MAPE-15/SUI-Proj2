#include "search-strategies.h"
#include <queue>
#include <set>
#include <map>
#include <algorithm>
#include "memusage.h"


namespace {

	/* Parent Map Type - parent links are used to reconstruct the solution plan: child -> (parent, action) */
	using ParentMap = std::map<SearchState, std::pair<SearchState, SearchAction>>;

	/**
	 * One frame in the iterative DFS "call stack"
	 *
	 * @param state: node (search state)
	 * @param actions: all applicable actions in the node
	 * @param next: index of the next action to try (default=0)
	 * @param depth: depth of the node (default=0)
	 */
	struct Frame {
		SearchState state;
		std::vector<SearchAction> actions;
		std::size_t next = 0;
		int depth = 0;
	};

	/**
	 * One OPEN entry for A* Search: state, path cost g, and total score f = g + h.
	 *
	 * @param state: node (search state)
	 * @param g: number of actions from the initial state (from the start) (default=0)
	 * @param f: function: g + h(state) (default=0.0)
	 */
	struct OpenEntry {
		SearchState state;
		int g = 0;
		double f = 0.0;
	};

	/**
	 * Min-heap: smallest f on top.
	 * Tie-break: prefer LARGER g when f ties. This usually expands fewer nodes.
	 */
	struct OpenCmp {
		bool operator()(const OpenEntry& a, const OpenEntry& b) const {
			if (a.f != b.f) return a.f > b.f;   // min-heap by f
			return a.g < b.g;                   // on tie: prefer deeper (larger g)
		}
	};

	/**
	 * @brief Rebuilds the action plan from the initial state to the given goal state
	 *
	 * Rebuild the action plan from init_state to `goal` using the parent map.
	 * Reconstructs the sequence of actions from init_state to 'goal' state.
	 * The map stores child -> (parent, action). It follows parents back to the root, collect actions in reverse, then flips once again - after that the plan is reconstructed
	 * This reconstruction  plan is used for BFS and A* Search
	 *
	 * @param goal The final goal state for which the plan should be reconstructed
	 * @param parent The map of explored states, mapping each child state to its parent state and the action that produced it
	 * @return A vector of "SearchAction" objects representing the sequence of actions from the initial state to the goal state (in correct execution order)
	 */
	std::vector<SearchAction> reconstruct_plan(const SearchState& goal, const ParentMap& parent) {
		std::vector<SearchAction> plan;
		const SearchState* cur = &goal;

		while (true) {
			auto it = parent.find(*cur);

			if (it == parent.end()) break;  // reached the root (no parent)

			plan.push_back(it->second.second);  // action that produced *cur
			cur = &it->second.first;  // step to parent (stored inside the map)
		}

		std::reverse(plan.begin(), plan.end());  // flip again
		return plan;  // final plan
	}

}  // namespace

/**
 * @brief Checks whether the current resident memory usage is getting too close to the limit.
 *
 * This helper is used by brute-force strategies (like BFS) to stop the search, before the external memory watcher kills the process.
 * It returns true, if current RSS + safety margin would exceed the given limit.
 *
 * @param limit_bytes  Hard upper bound on resident memory (in bytes). If 0, the check is disabled.
 * @param safety  Soft guard band (in bytes) kept below the limit to stop earlier (default: ~50 MiB).
 * @return true if memory usage is near/exceeds the limit (caller should abort gracefully), false otherwise.
 */
static bool nearMemLimit(const std::size_t limit_bytes, const std::size_t safety = (50ull << 20)) {

	if (!limit_bytes) return false;  // no limit configured - do not block

	const std::size_t rss = getCurrentRSS();  // current resident set size (bytes)

	// safe comparison without underflow.
	if (safety >= limit_bytes) {
		return rss >= limit_bytes;  // guard band exceeds the limit -> any non-zero RSS means “too close”.
	}

	// avoid overflow
	return rss + safety >= limit_bytes;  // true if we are within the guard band
}


std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {

	// if the initial state is already a goal, the optimal plan is empty - already solved
	if (init_state.isFinal()) return {};

	// BFS queue (FIFO) - states which are discovered but not yet expanded (queue OPEN)
	std::queue<SearchState> queue;

	// using set/map ordered by operator<, no need for hashing or operator==
	std::set<SearchState> visited_states;  // remembers all states which were already seen/visited (list CLOSED)
	ParentMap parent;  // child -> (parent, action_to_child)

	// initialization
	visited_states.insert(init_state);
	queue.push(init_state);

	// Main BFS Loop:
	// in BFS, the first time a goal is POPPED (from the queue) the shortest path has been found
	while (!queue.empty()) {

		// if approached a memory limit, return empty solution (clean exit)
		if (nearMemLimit(mem_limit_)) return {};

		// expand the next state in FIFO order (level by level)
		SearchState current = std::move(queue.front());
		queue.pop();

		// early success - if the popped state is a goal, reconstruct the final sequence of action going from start to goal
		if (current.isFinal()) return reconstruct_plan(current, parent);

		// generate all actions which are applicable in 'current' and try every successor
		const auto applicable_actions = current.actions();
		for (const auto& act : applicable_actions) {

			// build the successor by copying 'current' and executing the action
			SearchState next = current;
			if (!next.execute(act)) continue;  // skip if action can't be applicable

			// BFS invariant - discover each state once, if 'next' is new, record its parent.
			// optionally return the 'next' immediately if it's a goal and then enqueue it
			auto [_, inserted] = visited_states.insert(next);
			if (inserted) {
				parent.emplace(next, std::make_pair(current, act));

				// goal state has been discovered, reconstruct the plan immediately
				if (next.isFinal()) return reconstruct_plan(next, parent);

				queue.push(std::move(next));
			}
		}

	}

	// no solution within the explored space (or aborted due to memory guard)
	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {

    // if the initial state is already a goal, the optimal (shortest) plan is empty
    if (init_state.isFinal()) return {};

    // depth limit counts actions from the root - the root is at depth 0.
    const int max_depth = (depth_limit_ < 0) ? 0 : depth_limit_;

	// iterative DFS data
	std::vector<Frame> stack;			     // LIFO stack of frames
	std::vector<SearchAction> current_plan;  // actions along the CURRENT path only
	stack.reserve(1 << 12);

	// per-path cycle guard (NOT a memo table) - only states on the current recursion line.
	std::set<SearchState> recursion_guard;

	// initialize the search with the root frame.
	stack.push_back(Frame{init_state, init_state.actions(), 0, 0});
	recursion_guard.insert(init_state);

	// Main DFS Loop:
	while (!stack.empty()) {

		// if approached a memory limit, return empty solution (clean exit)
		if (nearMemLimit(mem_limit_)) return {};

		auto &[state, actions, next, depth] = stack.back();

		// goal test on the node at the top of the stack.
		if (state.isFinal()) return current_plan;

		// if depth limit has been reached or ran out of actions here - backtrack
		if (depth >= max_depth || next >= actions.size()) {
			recursion_guard.erase(state);
			stack.pop_back();

			if (!current_plan.empty()) current_plan.pop_back();  // drop action that led here

			continue;  // backtrack and restart the main loop
		}

		// expand the next successor from this node
		const SearchAction &action = actions[next++];
		SearchState child = state;
		if (!child.execute(action)) continue;  // skip invalid successors

		// avoid cycles on the current path
		if (recursion_guard.find(child) != recursion_guard.end()) continue;

		// descend one level deeper
		current_plan.push_back(action);
		if (child.isFinal()) return current_plan;  // quick exit if the goal state has been reached

		recursion_guard.insert(child);
		stack.push_back(Frame{child, child.actions(), 0, depth + 1});

	}

    // no solution within the explored space in certain depth (or aborted due to memory guard)
    return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {

	// trivial case - initial state is already a goal, return an empty plan
    if (init_state.isFinal()) return {};

    // queue OPEN (priority queue) and the book-keeping tables
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenCmp> open;

    // best known g for each state (ordered by operator< - no hashing used)
    std::map<SearchState, int> best_g;

    // parent links to reconstruct the plan: child -> (parent, action)
    ParentMap parent;

    // push the initial state
    const double h0 = compute_heuristic(init_state, *heuristic_);  // compute the heuristic
    open.push(OpenEntry{init_state, 0, h0});
    best_g.emplace(init_state, 0);

	// Main A* Search Loop:
    // expand states until OPEN is empty, or the program exits due to the memory guard
    while (!open.empty()) {

		// if approached a memory limit, return empty solution (clean exit)
        if (nearMemLimit(mem_limit_)) return {};

        OpenEntry cur = open.top();
        open.pop();

        // skip outdated queue entries (shorter path has already been found to this state)
		if (auto itg = best_g.find(cur.state); itg != best_g.end() && cur.g > itg->second) continue;

        // A* is optimally completed, when a goal state is POPPED from the OPEN queue - if so, return the optimal plan
        if (cur.state.isFinal()) return reconstruct_plan(cur.state, parent);

        // expand successors (unit cost per action)
        const auto actions = cur.state.actions();
        for (const auto& a : actions) {

        	// also inside the hot loop, if approached a memory limit, return empty solution (clean exit)
            if (nearMemLimit(mem_limit_)) return {};

            SearchState succ = cur.state;
            if (!succ.execute(a)) continue;

            const int tentative_g = cur.g + 1;

            // if 'succ' has never been seen, or a better path has been just found to it, record/update it.
	        if (auto it = best_g.find(succ); it == best_g.end() || tentative_g < it->second) {
                best_g[succ] = tentative_g;

                // IMPORTANT - overwrite parent (emplace would NOT replace older entry)
	        	parent.insert_or_assign(succ, std::pair{cur.state, a});

                const double h = compute_heuristic(succ, *heuristic_);
                open.push(OpenEntry{succ, tentative_g, tentative_g + h});
            }
        }
    }

	// no solution within the explored space with given limits (or aborted due to memory guard)
    return {};
}
