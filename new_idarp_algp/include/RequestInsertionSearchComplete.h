#pragma once

// TEMPORARY -- built only to measure whether RequestInsertionSearch's
// Pareto-front search for the transfer pattern regularly finds the true
// optimum, or misses it by discarding dominated task1 candidates too early.
// Delete this file and its .cpp once that comparison is done; nothing else
// depends on it.

#include "RequestInsertionSearch.h"
#include "Solution.h"

struct TransferSearchStats {
    int task1CandidatesGenerated = 0;  // PDP-filtered, across every entry stop and route
    int task1FeasibleFound = 0;        // how many of those passed the exact check
    int combinationsFeasible = 0;      // feasible (task1, task2) pairs actually compared
};

// Exhaustive (no Pareto front) search: verifies EVERY PDP-filtered task1
// candidate (every entry stop, every route, every position -- not just the
// non-dominated ones), and for each feasible one, every task2 candidate
// (every exit stop, every other route, every position). Returns the single
// best total-cost combination found (empty if none), plus counters for
// comparing against RequestInsertionSearch::findInsertions(..., useDropoffMetric).
// Not optimized for speed -- offline comparison only.
std::vector<RequestInsertionCandidate> findTransferInsertionComplete(const Solution& solution, int requestId);

// Runs all three transfer searches for `requestId` -- front (max shift),
// front (drop-off time), and the exhaustive reference -- prints a
// side-by-side comparison (costs, whether either front missed the true
// optimum, and the exhaustive search's counters), and returns the front
// (drop-off time) result, so this can be dropped into RequestInsertionSearch
// call sites as a temporary stand-in without changing their code shape.
std::vector<RequestInsertionCandidate> findTransferInsertionWrap(const Solution& solution, int requestId);
