# The biliardino ranking algorithms — the mathematics

Faithful to the code in this folder (`page_rank_biliardino_algorithm.py`,
`page_rank_biliardino_algorithm_bias.py`) and to the weighted matrix built
by the scoreboard. Open this file with VS Code's Markdown preview
(`Ctrl+Shift+V`) or on GitHub to see the formulas rendered.

## Setup: the win matrix

For $n$ teams, the **win matrix** $M \in \mathbb{R}^{n\times n}$ is

$$M_{ij} = \#\{\text{matches team } i \text{ won against team } j\},$$

so row $i$ holds team $i$'s wins and column $j$ holds team $j$'s losses.
Each win is a *link from the loser to the winner*: rank flows from the
defeated to the victor.

## 1. Classic PageRank

**Column normalization.** Each loser distributes its rank equally over
everyone who beat it. With column sums $c_j = \sum_i M_{ij}$ (the losses
of team $j$):

$$\widehat{M}_{ij} = \frac{M_{ij}}{\max(c_j,\,1)}$$

The $\max(\cdot,1)$ is the dangling-node guard: an **undefeated** team has
$c_j = 0$ and its column simply stays all-zero (see the deviations below).

**Fixed-point iteration.** With damping $d\in[0,1)$ (default $0.85$) and
start vector $v^{(0)} = \tfrac{1}{n}\mathbb{1}$:

$$v^{(k+1)} = d\,\widehat{M}\,v^{(k)} + \frac{1-d}{n}\,\mathbb{1},
\qquad \text{until } \lVert v^{(k+1)} - v^{(k)}\rVert_2 < 10^{-10}.$$

The limit is the unique solution of

$$v = d\,\widehat{M}\,v + \frac{1-d}{n}\,\mathbb{1}
\quad\Longleftrightarrow\quad
v = \frac{1-d}{n}\,\bigl(I - d\,\widehat{M}\bigr)^{-1}\,\mathbb{1},$$

which exists — and the iteration converges geometrically — because
$\lVert d\,\widehat{M}\rVert_1 \le d < 1$ makes the map a contraction.
Written per team:

$$v_i \;=\; \underbrace{\frac{1-d}{n}}_{\text{teleport floor}}
\;+\; d \sum_{j\,:\ i \text{ beat } j} \frac{v_j}{c_j}$$

> Team $i$'s strength is a base floor everyone gets, plus a damped share
> of every defeated opponent's strength, where each loser's strength is
> split evenly among all the teams that beat it. Beating a strong team
> that rarely loses pays far more than beating a team everyone beats.

**Deliberate deviations from textbook PageRank.**
1. Textbook PageRank replaces a dangling (all-zero) column with the
   uniform vector $\mathbb{1}/n$; here it stays zero, so an undefeated
   team absorbs rank without redistributing any — the total mass
   $\sum_i v_i$ ends up slightly below $1$.
2. No per-iteration renormalization — unnecessary, since the teleport
   term keeps $v$ positive and the contraction guarantees convergence.

## 2. Recursive deletion (the fully-connected-graph approach)

PageRank comparisons are only fair on a well-connected graph — a team
with a single lucky win distorts the flow. So before the *final*
standings, the least-connected teams are recursively disqualified.
(The live scoreboard deliberately does **not** apply this mid-tournament:
it would disqualify teams that simply haven't played yet.)

Define the **connectivity** of team $i$ as its games played,

$$g_i \;=\; \underbrace{\sum_j M_{ij}}_{\text{wins}}
\;+\; \underbrace{\sum_j M_{ji}}_{\text{losses}},$$

and let $D$ count the disqualified teams. Repeat (at most `n_steps`
times, default $n$):

1. **Stop test:** if every active team has the same connectivity,
   $g_i = g_{\max}$, **and** $g_{\max} = n - D$, declare the remaining
   graph fully connected and stop. Equal connectivity alone is not
   enough — it must also match the number of surviving teams, otherwise
   deletion continues.
2. **Prune:** mask dead teams ($g_i = 0 \mapsto n+1$ so they never win
   the argmin), find $g_{\min}$ among the rest, and disqualify **all**
   teams attaining it in one sweep:

$$\forall\, i \in \arg\min_i g_i:\qquad
M_{i,\cdot} \leftarrow 0,\qquad M_{\cdot,i} \leftarrow 0,$$

   incrementing $D$ by the number removed.

*Footnote:* for a strict everyone-played-everyone-once graph the natural
stop target would be $g_{\max} = (n - D) - 1$ (nobody plays themselves);
the implementation tests $g_{\max} = n - D$, which is only reachable when
at least one pair has played more than once.

## 3. Bias PageRank (the live scoreboard's default)

Identical flow structure, but the flat teleport $\mathbb{1}/n$ is
replaced by a **participation baseline**: with games played
$g_i$ as above (computed on the *raw* matrix),

$$p_i = \frac{g_i}{\sum_k g_k},$$

the iteration becomes

$$v^{(k+1)} = d\,\widehat{M}\,v^{(k)} + (1-d)\,p .$$

The teleport floor is no longer equal for everyone — it is proportional
to how much a team has played. Sitting out earns nothing; playing (even
losing) earns baseline credit. This is why the leave-one-out match
impacts often show a *positive* delta for the loser: the participation
credit of one more match can outweigh the rank flow lost.

## 4. Weighted bias PageRank

Same algorithm as §3, run on a **goal-difference-weighted matrix** $W$
instead of the binary $M$. A match between $A$ and $B$ with game totals
$T_A, T_B$ over $G$ games adds, to the winner's row,

$$W_{\text{winner},\,\text{loser}} \mathrel{+}= 1 + x\,\frac{\lvert T_A - T_B\rvert}{10\,G},
\qquad x \in [1, 10).$$

The constant $1$ is the base credit for the win itself; the second term
rewards domination, tuned by the hyperparameter $x$. The base credit is
essential: a *pure* scaling weight $x\,\lvert T_A-T_B\rvert/(10G)$ would
cancel entirely, because both the column normalization
$\widehat{W}_{ij} = W_{ij}/\max(c_j,1)$ **and** the participation
baseline $p$ are ratios — multiplying every edge by the same constant
changes neither. Only the *relative* spread between narrow and dominant
wins survives, which is exactly what $x$ controls: at small $x$ the
ranking hugs plain bias PageRank; at large $x$, sweeping 10–0 outweighs
grinding out 10–9s.

A match with no recorded game scores contributes the base credit $1$
only. Matches, $d$ and $x$ are wired through `compute_scores.py`; the
two algorithm files remain the single source of truth for the math.
