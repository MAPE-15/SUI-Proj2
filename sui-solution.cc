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

	// Trivial: already solved
    if (init_state.isFinal()) return {};

    // Depth limit counts actions from root; root has depth 0
    const int max_depth = (depth_limit_ < 0) ? 0 : depth_limit_;

    // One stack frame for iterative DFS
    struct Frame {
        SearchState state;
        std::vector<SearchAction> actions;
        std::size_t next = 0;
        int depth = 0;
    };

    // Explicit stack and current action path only (NO global visited, NO parent map)
    std::vector<Frame> stack;
    std::vector<SearchAction> path;
    stack.reserve(1 << 12);

    // Per-path cycle guard: contains only states on the CURRENT path (not a global visited!)
    std::set<SearchState> on_path;

    // Init
    stack.push_back(Frame{init_state, init_state.actions(), 0, 0});
    on_path.insert(init_state);

    while (!stack.empty()) {
        // Optional graceful abort near mem limit (recommended)
        if (nearMemLimit(mem_limit_)) return {};

        Frame &f = stack.back();

        // Goal test
        if (f.state.isFinal()) return path;

        // Depth limit / no more actions -> backtrack
        if (f.depth >= max_depth || f.next >= f.actions.size()) {
            on_path.erase(f.state);
            stack.pop_back();
            if (!path.empty()) path.pop_back();   // remove action that led here
            continue;
        }

        // Expand next successor
        const SearchAction &a = f.actions[f.next++];
        SearchState child = f.state;              // copy-construct (assignment is disabled)
        if (!child.execute(a)) continue;

        // Prevent cycles on the current path only (still no global memoization)
        if (on_path.find(child) != on_path.end()) continue;

        // Descend
        path.push_back(a);
        if (child.isFinal()) return path;        // fast exit
        on_path.insert(child);
        stack.push_back(Frame{child, child.actions(), 0, f.depth + 1});
    }

    // No solution found within given depth/memory constraints
    return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	 // Trivial: už je cieľ
    if (init_state.isFinal()) return {};

    // Pomocná štruktúra pre OPEN (min-heap podľa f=g+h, pri rovnosti menšie g)
    struct Node {
        SearchState state;
        int g = 0;           // dĺžka cesty (počet akcií)
        double f = 0.0;      // g + h
    };
    struct Cmp {
        bool operator()(const Node &a, const Node &b) const {
            if (a.f != b.f) return a.f > b.f; // min-heap (väčšie f má nižšiu prioritu)
            return a.g > b.g;                 // pri rovnosti preferuj plytšie riešenia
        }
    };

    // OPEN fronta a tabuľky
    std::priority_queue<Node, std::vector<Node>, Cmp> open;

    // Najlepšie známe g(s) pre daný stav (kľúče porovnávané cez operator<)
    std::map<SearchState, int> g_score;

    // Rodič → na rekonštrukciu plánu (dieťa -> (rodič, akcia))
    std::map<SearchState, std::pair<SearchState, SearchAction>> parent;

    // Heuristika pre počiatočný stav
    const double h0 = compute_heuristic(init_state, *heuristic_);
    open.push(Node{init_state, 0, 0.0 + h0});
    g_score.emplace(init_state, 0);

    // Rekonštrukcia akcií z parent mapy
    auto reconstruct = [&](const SearchState &goal) {
        std::vector<SearchAction> path;
        const SearchState *cur = &goal;
        while (true) {
            auto it = parent.find(*cur);
            if (it == parent.end()) break;
            path.push_back(it->second.second);
            cur = &it->second.first;
        }
        std::reverse(path.begin(), path.end());
        return path;
    };

    while (!open.empty()) {
        // Voliteľná ochrana pamäte – ak sme blízko limitu, vráť prázdne riešenie
        if (nearMemLimit(mem_limit_)) return {};

        Node cur = std::move(open.top());
        open.pop();

        // "Outdated queue entry" ochrana: preskoč, ak už máme lepšie g v tabuľke
        auto it_g = g_score.find(cur.state);
        if (it_g != g_score.end() && cur.g > it_g->second) continue;

        // Cieľ test – pri A* je optimálne ukončiť, keď cieľ POP-neme z OPEN
        if (cur.state.isFinal()) return reconstruct(cur.state);

        // Rozbaľ susedov
        const auto actions = cur.state.actions();
        for (const auto &a : actions) {
            SearchState succ = cur.state;   // copy-construct (assign je zakázaný)
            if (!succ.execute(a)) continue;

            const int tentative_g = cur.g + 1; // jednotkový náklad za akciu

            // Ak nemáme g(succ) alebo tentatívne g je lepšie, aktualizuj
            auto it = g_score.find(succ);
            if (it == g_score.end() || tentative_g < it->second) {
                g_score[succ] = tentative_g;
                parent.emplace(succ, std::make_pair(cur.state, a));

                // Heuristika – zadanie povoľuje 2 varianty; "student" vracia 0 → A* == BFS
                const double h = compute_heuristic(succ, *heuristic_);
                open.push(Node{succ, tentative_g, tentative_g + h});
            }
        }
    }

    // Nenašli sme riešenie v daných hraniciach (alebo sme skončili kvôli pamäti)
    return {};
}
