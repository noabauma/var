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



def pagerank(M, d=0.85, participation_bias=True):
    M = M.copy()
    N = M.shape[1]

    # games played = wins (row) + losses (column), on the RAW matrix
    games_played = M.sum(axis=0) + M.sum(axis=1)

    if participation_bias and games_played.sum() > 0:
        p = games_played / games_played.sum()   # baseline ∝ participation
    else:
        p = np.ones(N) / N                       # uniform (your old behaviour)

    column_sums = M.sum(axis=0)
    column_sums[column_sums == 0] = 1
    M = M / column_sums

    w = np.ones(N) / N
    M_hat = d * M
    v = M_hat @ w + (1 - d) * p                  # <-- p replaces flat 1/N
    while np.linalg.norm(w - v) >= 1e-10:
        w = v
        v = M_hat @ w + (1 - d) * p
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
    
    M, disqualified_teams = recursive_deletion(M, 1)

    v = pagerank(M, 0.85)

    print("Group B:\n", v)
