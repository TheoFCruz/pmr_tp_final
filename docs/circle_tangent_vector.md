# Tangent Vector to a Circular Velocity Obstacle

This note derives the tangent vector from an ego agent center to the inflated
boundary of a neighboring circular agent. This is useful for defining the
velocity-obstacle collision cone.

Let the ego agent center be the origin, and let the relative position from the
ego agent to the neighbor be

```text
p = p_j - p_i
```

Define

```text
d = ||p||
e = p / d
R = r_i + r_j
```

where `R` is the inflated obstacle radius, obtained by adding the two agent
radii.

The vector `e` points from the ego center to the neighbor center. A perpendicular
unit vector is

```text
e_perp = [-e_y, e_x]
```

The other perpendicular direction is `-e_perp`.

We want the tangent vector `l` from the ego center to the inflated circle. The
triangle formed by

```text
ego center -> neighbor center -> tangent point
```

is right-angled at the tangent point. Therefore, if

```text
L = ||l||
```

then

```text
d² = L² + R²
```

so

```text
L = sqrt(d² - R²)
```

This requires `d > R`. If `d <= R`, the inflated circles overlap and the tangent
is not defined.

Now decompose the tangent vector in the local basis aligned with `p`:

```text
l = a e + b e_perp
```

Let `gamma` be the angle between `p` and either tangent ray. From the right
triangle,

```text
cos(gamma) = L / d
sin(gamma) = R / d
```

Since `||l|| = L`, the components of `l` are

```text
a = L cos(gamma) = L * L / d = L² / d
b = L sin(gamma) = L * R / d = R L / d
```

Therefore, the two tangent vectors are

```text
l_plus  = (L² / d) e + (R L / d) e_perp
l_minus = (L² / d) e - (R L / d) e_perp
```

These two vectors are symmetric around `p` and define the two sides of the
velocity-obstacle cone. The cone half-angle is

```text
gamma = asin(R / d)
```

or equivalently

```text
gamma = atan2(R, L)
```

It is tempting to write the tangent vector as

```text
l = p + R e_perp
```

but this is not tangent to the circle. The reason is that the radius from the
neighbor center to the tangent point is perpendicular to `l`, not perpendicular
to `p`.

The tangent vector can be written as

```text
l = p + R n
```

but the surface normal `n` at the tangent point is not simply `e_perp`. Instead,

```text
n_plus  = -(R / d) e + (L / d) e_perp
n_minus = -(R / d) e - (L / d) e_perp
```

so that

```text
l_plus  = p + R n_plus
l_minus = p + R n_minus
```

Substituting `p = d e` gives

```text
l_plus = d e + R [-(R / d) e + (L / d) e_perp]
       = (d - R² / d) e + (R L / d) e_perp
       = ((d² - R²) / d) e + (R L / d) e_perp
       = (L² / d) e + (R L / d) e_perp
```

and similarly for `l_minus`.
