import numpy as np

# Set print options
np.set_printoptions(precision=4, suppress=True, linewidth=100)

def recursive_deletion(M, n_steps = -1):
    """An algorithm to recrusively deleting the teams until a fully connected graph exists

    Args:
        M (numpy array): adjacency matrix where M_i,j represents the link from 'j' to 'i', such that for all 'j'
        n_steps (int): Instead of doing this algorithm until a fully connected graph one can also do it by number of steps
    
    Return:
        M matrix with the disqualified teams
    """
    assert M.shape[0] == M.shape[1], "This is not a square matrix"

    n_teams = M.shape[0]
    disqualified_teams = 0

    # If the number of steps is not set then there is at most n_teams iterations to do
    if n_steps == -1:
        n_steps = n_teams

    for step in range(n_steps):
        # Compute the sum of each column
        column_sums = M.sum(axis=0)

        # Compute the sum of each row
        row_sums = M.sum(axis=1)

        num_games_per_team = row_sums + column_sums

        print("Number of played matches: ", num_games_per_team, " after iter: ", step)

        # Stop when all teams have played the same number of games
        if np.all(num_games_per_team == np.max(num_games_per_team)):
            # It is only fully connected if the this also true (else continue disqualifying)
            if np.max(num_games_per_team) == (n_teams - disqualified_teams):
                print("We found a fully connected graph!")
                break
            else:
                print("We found same number of won teams across all still existing teams! (Continue searching)")
                continue

        # Set the zero value arbitrary high to not count it
        num_games_per_team[num_games_per_team == 0] = n_teams + 1
        
        # Find the minimum value
        min_value = np.min(num_games_per_team)

        # Find all indices where the minimum value occurs
        indices = np.where(num_games_per_team == min_value)[0]

        # count number of disqualified teams
        disqualified_teams += len(indices)

        for index in indices:
            M[:, index] = np.zeros(n_teams)
            M[index, :] = np.zeros(n_teams)

    # Compute the sum of each column
    column_sums = M.sum(axis=0)

    # Compute the sum of each row
    row_sums = M.sum(axis=1)

    num_games_per_team = row_sums + column_sums

    print("Number of played matches: ", num_games_per_team, " after iter: ", n_steps)

    return M, disqualified_teams






def print_win_table(M, team_names=None, title=None):
    """Print a table of how many times the row team beat the column team.

    Since M_i,j represents the link from 'j' to 'i' (i.e. team 'i' won
    against team 'j'), entry (row=i, col=j) is exactly the number of wins
    of team 'i' against team 'j'.

    Args:
        M (numpy array): adjacency / win matrix
        team_names (list[str], optional): labels for the teams; defaults to
            'T0', 'T1', ...
        title (str, optional): heading printed above the table
    """
    n = M.shape[0]
    if team_names is None:
        team_names = [f"T{i}" for i in range(n)]
    assert len(team_names) == n, "Number of team names must match matrix size"

    W = M.astype(int)

    # Column width fits the widest label / count (min 2 for readability)
    col_w = max(2, max(len(name) for name in team_names), len(str(W.max())))
    corner_w = max(len(name) for name in team_names)

    if title:
        print(title)
    print("(rows = winner, columns = loser)")

    header = " " * corner_w + " | " + " ".join(f"{name:>{col_w}}" for name in team_names)
    print(header)
    print("-" * len(header))
    for i in range(n):
        cells = " ".join(f"{W[i, j]:>{col_w}}" for j in range(n))
        print(f"{team_names[i]:>{corner_w}} | {cells}   (won {W[i].sum()})")
    print()


def pagerank(M, d: float = 0.85):
    """PageRank algorithm with explicit number of iterations. Returns ranking of nodes (pages) in the adjacency matrix.

    Parameters
    ----------
    M : numpy array
        adjacency matrix where M_i,j represents the link from 'j' to 'i', such that for all 'j'
    d : float, optional
        damping factor, by default 0.85

    Returns
    -------
    numpy array
        a vector of ranks such that v_i is the i-th rank from [0, 1],

    """
    # # Compute the sum of each column
    # column_sums = M.sum(axis=0)

    # # Compute the sum of each row
    # row_sums = M.sum(axis=1)

    # # Teams with more number of won matches have a higher score
    # np.fill_diagonal(M, row_sums + column_sums)

    print(M)

    # Compute the sum of each column
    column_sums = M.sum(axis=0)

    # Ignore column full of zeros
    column_sums[column_sums == 0] = 1

    # Scale each column such that the sum is 1
    M /= column_sums

    N = M.shape[1]
    w = np.ones(N) / N
    M_hat = d * M
    v = M_hat @ w + (1 - d) / N
    while(np.linalg.norm(w - v) >= 1e-10):
        w = v
        v = M_hat @ w + (1 - d) / N
    return v

if __name__ == "__main__":

    # M = np.array([[0, 0, 0, .25],
    #             [0, 0, 0, .5],
    #             [1, 0.5, 0, .25],
    #             [0, 0.5, 1, 0]])
    
    # group A
    M = np.array([[0, 0, 1, 1, 1, 1, 1, 1, 1],
                  [1, 0, 1, 1, 1, 1, 0, 0, 1],
                  [0, 0, 0, 1, 1, 0, 1, 1, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 1],
                  [0, 0, 0, 1, 0, 0, 0, 0, 0],
                  [0, 0, 1, 1, 0, 0, 1, 0, 1],
                  [0, 0, 0, 1, 0, 0, 0, 0, 0],
                  [0, 0, 0, 1, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0]], dtype=np.float64)

    print_win_table(M, title="Group A — head-to-head wins")

    M, disqualified_teams = recursive_deletion(M, 2)

    v = pagerank(M, 0.85)

    print("Group A:\n", v)

    # group B
    M = np.array([[0, 0, 0, 0, 0, 0, 1, 1, 0],
                  [1, 0, 1, 1, 0, 0, 0, 1, 0],
                  [1, 0, 0, 0, 0, 0, 0, 0, 0],
                  [1, 0, 1, 0, 0, 0, 1, 1, 0],
                  [0, 1, 1, 1, 0, 0, 1, 1, 1],
                  [1, 1, 1, 0, 1, 0, 1, 1, 1],
                  [0, 1, 1, 0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 1, 0, 0]], dtype=np.float64)

    print_win_table(M, title="Group B — head-to-head wins")

    M, disqualified_teams = recursive_deletion(M, 1)

    v = pagerank(M, 0.85)

    print("Group B:\n", v)
