#set page(
  numbering: "1",
)
#set heading(numbering: "1.")
#set math.equation(numbering: "(1)")
#show link: underline

#let bx = $bold(x)$
#let bmu = $bold(mu)$
#let bSigma = $bold(Sigma)$
#let bu = $bold(u)$

#align(center, text(17pt)[
  *Multivariate Gaussian function and Normal distribution transform*
])

#outline()

= Multivariate Gaussian function

Multivariate Gaussian function:

$
cal(N) (bx|bmu, bSigma^2) =
1 / ((2 pi)^(D/2)) 1 / (|bSigma|^(1/2))
exp {-1 / 2 (bx - bmu)^upright(T) bSigma^(-1) (bx - bmu) }
$
where $bx$ is the input vector variable, $bmu$ is the mean vector and $bSigma$ is the covariant matrix.

== Transformation from Gaussian space

How do we find the transformation to the coordinate where this function becomes a unit Gaussian, i.e. $bSigma = I$?
We name the transformation $T$, then it should hold:

$
cal(N)(T(bx)|bmu, bSigma^2) = cal(N)(bx| 1, 1)
$

Looking inside the parameter, this should hold:

$
-1/2 {T(bx) - mu}^upright(T) bSigma^(-1) {T(bx) - bmu} = bx^upright(T) I bx
$<eq:param>

But this is rather difficult way to look at the problem.
We want to reduce the problem just about $bSigma$, so we first define a variable $bx_mu equiv bx - mu$.
Then we can say

$
-1/2 bx_mu^upright(T) bSigma^(-1) bx_mu = bx^upright(T) I bx
$<eq:inner>

Now, if we find the eigenvalues $lambda_i$ and eigenvectors $bu_i$ of $bSigma$ (how to do it is written in about thousand books, so I would not reiterate), we can rewrite it as

$
bSigma = sum_(i=1)^M 1 / lambda_i bu_i bu_i^upright(T).
$<eq:diag>

This is called diagonalization, because if you define a diagonal matrix $Lambda$ whose $i$-th row and column has $lambda_i$ and 0 elsewhere,
you can write it in this way too:
$
bSigma = U Lambda U
$

However, for the purpose of our discussion, @eq:diag is easier to handle.
Now, substituting it to @eq:inner will yield:

$
-1/2 bx_mu^upright(T) sum_(i=1)^M 1 / lambda_i bu_i bu_i^upright(T) bx_mu = bx^upright(T) I bx
$

You can rearrange the formula to see that it is just dot products between vectors.

$
-1/2 sum_(i=1)^M 1 / lambda_i ( bx_mu^upright(T)  bu_i ) ( bu_i^upright(T) bx_mu ) \
= -1/2 sum_(i=1)^M 1 / lambda_i ( bx_mu^upright(T)  bu_i )^2
$

So you can imagine it is a _re-projection_ of coordinate system using eigenvectors $bu_i$.

== Solving Eigenvalues Problem Programatically

Now we need to solve the eigenvalues problem.
If we use a software package, it would be easy, but our goal here is to implement everything by ourselves.

First, let's recap the eigenvalue problem.
Let $A$ be the matrix we want to obtain eigenvalues and eigenvectors for.
Then it should satisfy this with eigenvalue $lambda$ and eigenvector $x$.

$
A x = lambda x
$

Rewriting the formula like below obtains the characteristic equation.

$
(lambda I - A) x = 0
$<eq:cheq>

Let's assume 2 dimension case, in which $A$ can be represented element-wise as

$
A = mat(a, b; c, d).
$

With this, we can write @eq:cheq as:

$
mat(a - lambda, b; c, d - lambda) mat(x; y) = 0
$

This is a system of linear equations.
So we can determine if it has a solution by the determinant

$
det(A - lambda I) = 0
$ <eq:det-lambda>

In 2D case, rearranging @eq:det-lambda obtains this

$
(a - lambda) (d - lambda) - b c = 0 \
lambda^2 - (a + d) lambda - b c = 0
$

Solving this for $lambda$ yields:

$
lambda = (a + d plus.minus sqrt((a + d)^2 - 4 b c)) / 2
$ <eq:eigenv>

Now we can write the code to find eivengalues like below:

```rust
fn discriminant([a, b, c, d]: &[f64; 4]) -> f64 {
    (a + d).powi(2) - 4. * (a * d - b * c)
}

fn find_eigenvalues(mat: &[f64; 4]) -> Option<[f64; 2]> {
    let disc = discriminant(mat);
    if disc < 0. {
        return None;
    }

    let trace = mat[0] + mat[3];
    let lambda1 = (trace + disc.sqrt()) / 2.;
    let lambda2 = (trace - disc.sqrt()) / 2.;
    Some([lambda1, lambda2])
}
```

The next step is to obtain eigenvectors. We got 2 eigenvalues for 2D (as @eq:eigenv) and each of them has its own eigenvector.

Let's call the positive eigenvalue $lambda_1$.
Substituting it to @eq:cheq will yield

$
(A - lambda_1 I) x = 0
$

which can be written element-wise as:

$
mat(a - lambda_1, b; c, d - lambda_1) mat(x_1; y_1) = 0
$

By fixing the value $x_1 equiv 1$, we can solve this equation against $y_1$.

$
mat(a - lambda_1, b; c, d - lambda_1) mat(1; y_1) = 0 \
cases(a - lambda_1 + b y_1 = 0, c + (d - lambda) y_1 = 0) \
cases( y_1 = (- a + lambda_1) / b, y_1 = -c / (d - lambda))
$


== Alternative approach

This is rather complicated to solve in general, so let's solve it in 2 dimension case.

$
bSigma = mat(
    sigma_(00), sigma_(01) ;
    sigma_(10), sigma_(11)
), bmu = mat(mu_y; mu_y)
$

Let's calculate the inverse of it:

$
bSigma^(-1) = 1/(sigma_00 sigma_11 - sigma_01 sigma_10)
mat(
  sigma_11, -sigma_01 ;
  -sigma_10, sigma_00
)
$

Here, the right hand side of @eq:param is calculated as:

$
bx^upright(T) I bx = x^2 + y^2
$

We call the X and Y components of $T(bx)$ as $T_x$ and $T_y$, respectively for brevity.
Also we define $x_t equiv T_x - mu_x, y_t equiv T_y - mu_y$.

Then the left hand side of equation @eq:param will be:

$
x^2 + y^2 = -1/2 mat(x_t, y_t) mat(sigma_00, sigma_01; sigma_10, sigma_11)^(-1) mat(x_t; y_t)
$

Let's define the determinant of $bSigma$ since it shows up over and over again.
$det(bSigma) = 1/(sigma_00 sigma_11 - sigma_01 sigma_10)$

$
x^2 + y^2 = -det(bSigma)/2
  mat(x_t sigma_11 - y_t sigma_10, space.quad - x_t sigma_01 + y_t sigma_00) mat(x_t; y_t) \
= -det(bSigma)/2
  (x_t^2 sigma_11 - x_t y_t sigma_10 - y_t x_t sigma_01 + y_t^2 sigma_00) \
= -det(bSigma)/2
  (x_t^2 sigma_11 - x_t y_t (sigma_10 + sigma_01) + y_t^2 sigma_00)
$
