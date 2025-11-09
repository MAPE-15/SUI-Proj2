#include "search-strategies.h"
#include <queue>
#include <set>
#include <map>
#include <algorithm>
#include "memusage.h"


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
static inline bool nearMemLimit(std::size_t limit_bytes, std::size_t safety = (50ull << 20)) {
	if (!limit_bytes) return false;          // no limit configured - do not block
	std::size_t rss = getCurrentRSS();       // current resident set size (bytes)
	return rss + safety >= limit_bytes;      // true if we are within the guard band
}

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	// trivial case: already solved - empty plan.
	if (init_state.isFinal()) return {};

	// BFS FIFO queue of states discovered but not yet expanded.
	std::queue<SearchState> q;

	// Visited set and back-pointers:
	// We use std::set/std::map (ordered by operator<), so we don't need hashing or operator==.
	std::set<SearchState> visited;  // tracks states we have already generated
	std::map<SearchState, std::pair<SearchState, SearchAction>> parent; // child → (parent, action)

	// Initialize the search.
	visited.insert(init_state);
	q.push(init_state);

	// Reconstructs the action sequence from init_state to 'goal' by following the parent map.
	// Note: we follow references stored inside 'parent' to avoid copy-assignment of SearchState.
	auto reconstruct = [&](const SearchState& goal) {
		std::vector<SearchAction> path;
		const SearchState* cur = &goal;

		while (true) {
			auto it = parent.find(*cur);
			if (it == parent.end()) break;         // reached the initial state
			path.push_back(it->second.second);     // action used to enter *cur
			cur = &it->second.first;               // step to the parent state
		}

		std::reverse(path.begin(), path.end());     // actions were collected backwards
		return path;
	};

	// Standard BFS loop: expand by increasing depth; first time we pop a goal is optimal.
	while (!q.empty()) {

		// Optional safeguard: stop early if we approach the configured memory limit.
		if (nearMemLimit(mem_limit_)) return {};

		// Take the next state in FIFO order.
		SearchState s = std::move(q.front());
		q.pop();

		// If this state is already a goal, we are guaranteed to have a shortest plan.
		if (s.isFinal()) return reconstruct(s);

		// Generate successors: actions() enumerates applicable moves from 's'.
		const auto actions = s.actions();
		for (const auto& a : actions) {
			// Successor 't' is created by copying 's' and applying action 'a' in-place.
			SearchState t = s;                  // copy-construct (copy-assignment is disabled)
			if (!t.execute(a)) continue;        // skip if not applicable (defensive)

			// BFS invariant: mark visited upon first discovery; never enqueue duplicates.
			auto ins = visited.insert(t);
			if (ins.second) {                   // inserted == true → first time we see 't'
				parent.emplace(t, std::make_pair(s, a));  // remember how we reached 't'
				if (t.isFinal()) return reconstruct(t);   // fast exit if goal discovered
				q.push(std::move(t));           // enqueue for future expansion
			}
		}

	}

	// No solution within explored space / aborted due to memory guard.
	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	return {};
}
