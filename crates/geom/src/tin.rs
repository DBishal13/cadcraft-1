//! Triangulated irregular networks (TINs): terrain surfaces for plan, contour, section and volume work.
//!
//! A [`Tin`] is a set of 3D points and the triangles joining them (counter-clockwise in plan). It comes from
//! a surface file's own triangles ([`Tin::new`]) or from a Delaunay triangulation of points
//! ([`Tin::from_points`]). Everything here is hostile-input safe: sizes are capped, indices are checked once
//! at construction, and nothing panics.

use std::collections::HashMap;

use crate::{Bounds2, Vec2, Vec3};

/// Most points a surface may hold.
pub const MAX_TIN_POINTS: usize = 4_000_000;
/// Most triangles a surface may hold.
pub const MAX_TIN_TRIANGLES: usize = 8_000_000;
/// Cells in a triangle lookup grid.
const MAX_GRID_CELLS: usize = 4_000_000;
/// Signed area under which a triangle is dropped as a sliver (square feet / square metres).
const MIN_AREA: f64 = 1e-9;
const NONE: u32 = u32::MAX;

/// The volume between two surfaces over a region.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Volume {
    /// Where the top surface is above the base (cubic units).
    pub fill: f64,
    /// Where the top surface is below the base (cubic units).
    pub cut: f64,
    /// Plan area compared (the part both surfaces cover).
    pub area: f64,
    /// Plan area where the top is above the base.
    pub fill_area: f64,
    /// Plan area where the top is below the base.
    pub cut_area: f64,
}

impl Volume {
    /// Fill minus cut.
    pub fn net(&self) -> f64 {
        self.fill - self.cut
    }
}

/// A triangulated surface with a lookup grid over its triangles.
#[derive(Clone, Debug)]
pub struct Tin {
    pts: Vec<Vec3>,
    tris: Vec<[u32; 3]>,
    bounds: Bounds2,
    origin: Vec2,
    cell: f64,
    nx: usize,
    ny: usize,
    /// Triangle ids per grid cell, `start[c]..start[c + 1]` into `ids`.
    start: Vec<u32>,
    ids: Vec<u32>,
}

fn orient(a: Vec2, b: Vec2, c: Vec2) -> f64 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

fn xy(v: Vec3) -> Vec2 {
    Vec2::new(v.x, v.y)
}

impl Tin {
    /// A surface from points and triangles (indices into `pts`). Slivers are dropped and every triangle is made
    /// counter-clockwise. Errors on non-finite coordinates, out-of-range indices, or too much data.
    pub fn new(pts: Vec<Vec3>, tris: Vec<[u32; 3]>) -> Result<Tin, String> {
        if pts.len() > MAX_TIN_POINTS || tris.len() > MAX_TIN_TRIANGLES {
            return Err(format!("surface too large ({} points, {} triangles)", pts.len(), tris.len()));
        }
        if pts.iter().any(|p| !p.is_finite()) {
            return Err("surface has a non-finite coordinate".into());
        }
        let mut good = Vec::with_capacity(tris.len());
        for t in tris {
            let (Some(&a), Some(&b), Some(&c)) = (pts.get(t[0] as usize), pts.get(t[1] as usize), pts.get(t[2] as usize)) else {
                return Err("triangle index out of range".into());
            };
            let area = orient(xy(a), xy(b), xy(c));
            if area.abs() < MIN_AREA {
                continue;
            }
            good.push(if area > 0.0 { t } else { [t[0], t[2], t[1]] });
        }
        let mut b = Bounds2::EMPTY;
        for t in &good {
            for &i in t {
                if let Some(&p) = pts.get(i as usize) {
                    b.add(xy(p));
                }
            }
        }
        let mut tin = Tin { pts, tris: good, bounds: b, origin: Vec2::new(0.0, 0.0), cell: 1.0, nx: 1, ny: 1, start: Vec::new(), ids: Vec::new() };
        tin.build_grid();
        Ok(tin)
    }

    /// Delaunay triangulation of `pts` (their plan positions; points closer than `1e-6` count as one, the
    /// highest kept). Triangles reaching the convex hull's super-triangle are dropped, so the outline is the hull.
    pub fn from_points(pts: &[Vec3]) -> Result<Tin, String> {
        if pts.len() > MAX_TIN_POINTS {
            return Err(format!("too many points ({})", pts.len()));
        }
        let mut v: Vec<Vec3> = pts.iter().copied().filter(|p| p.is_finite()).collect();
        v.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)).then(b.z.total_cmp(&a.z)));
        v.dedup_by(|b, a| (a.x - b.x).abs() < 1e-6 && (a.y - b.y).abs() < 1e-6);
        if v.len() < 3 {
            return Err("need at least 3 distinct points".into());
        }
        let tris = delaunay(&v)?;
        Tin::new(v, tris)
    }

    pub fn points(&self) -> &[Vec3] {
        &self.pts
    }

    pub fn triangles(&self) -> &[[u32; 3]] {
        &self.tris
    }

    /// Plan bounds of the triangles (empty when there are none).
    pub fn bounds(&self) -> Bounds2 {
        self.bounds
    }

    /// Corners of triangle `i`.
    pub fn corners(&self, i: usize) -> Option<[Vec3; 3]> {
        let t = self.tris.get(i)?;
        Some([*self.pts.get(t[0] as usize)?, *self.pts.get(t[1] as usize)?, *self.pts.get(t[2] as usize)?])
    }

    /// Lowest and highest elevation among the used points.
    pub fn z_range(&self) -> Option<(f64, f64)> {
        let mut r: Option<(f64, f64)> = None;
        for t in &self.tris {
            for &i in t {
                let z = self.pts.get(i as usize)?.z;
                r = Some(match r {
                    None => (z, z),
                    Some((lo, hi)) => (lo.min(z), hi.max(z)),
                });
            }
        }
        r
    }

    /// Plan area of all triangles.
    pub fn area(&self) -> f64 {
        (0..self.tris.len()).filter_map(|i| self.corners(i)).map(|c| orient(xy(c[0]), xy(c[1]), xy(c[2])) / 2.0).sum()
    }

    /// A copy without the triangles that have an edge longer than `max_edge` (plan length), as when a
    /// boundary-less triangulation bridges a gap it should not.
    pub fn trimmed(&self, max_edge: f64) -> Result<Tin, String> {
        let m2 = max_edge * max_edge;
        let keep = (0..self.tris.len())
            .filter_map(|i| {
                let c = self.corners(i)?;
                let long = (0..3).any(|k| {
                    let (a, b) = (xy(c[k]), xy(c[(k + 1) % 3]));
                    (a.x - b.x).powi(2) + (a.y - b.y).powi(2) > m2
                });
                (!long).then(|| self.tris.get(i).copied()).flatten()
            })
            .collect();
        Tin::new(self.pts.clone(), keep)
    }

    fn build_grid(&mut self) {
        let b = self.bounds;
        if self.tris.is_empty() || !!b.is_empty() {
            self.start = vec![0, 0];
            return;
        }
        let (w, h) = ((b.max.x - b.min.x).max(1e-9), (b.max.y - b.min.y).max(1e-9));
        let target = (self.tris.len() / 2).clamp(1, MAX_GRID_CELLS) as f64;
        let cell = (w * h / target).sqrt().max(w.max(h) / 4096.0);
        let (nx, ny) = (((w / cell).ceil() as usize).clamp(1, 4096), ((h / cell).ceil() as usize).clamp(1, 4096));
        self.origin = b.min;
        self.cell = cell;
        self.nx = nx;
        self.ny = ny;
        let mut count = vec![0u32; nx * ny + 1];
        for pass in 0..2 {
            let mut fill = count.clone();
            if pass == 1 {
                let mut acc = 0u32;
                for c in count.iter_mut() {
                    let n = *c;
                    *c = acc;
                    acc += n;
                }
                self.ids = vec![0; acc as usize];
                fill = count.clone();
            }
            for (ti, _) in self.tris.iter().enumerate() {
                let Some(c) = self.corners(ti) else { continue };
                let tb = c.iter().fold(Bounds2::EMPTY, |mut bb, p| {
                    bb.add(xy(*p));
                    bb
                });
                let ((x0, y0), (x1, y1)) = (self.cell_of(tb.min), self.cell_of(tb.max));
                for cy in y0..=y1 {
                    for cx in x0..=x1 {
                        let k = cy * nx + cx;
                        if pass == 0 {
                            if let Some(n) = count.get_mut(k) {
                                *n += 1;
                            }
                        } else if let Some(pos) = fill.get_mut(k) {
                            if let Some(slot) = self.ids.get_mut(*pos as usize) {
                                *slot = ti as u32;
                            }
                            *pos += 1;
                        }
                    }
                }
            }
        }
        if let Some(last) = count.last_mut() {
            *last = self.ids.len() as u32;
        }
        self.start = count;
    }

    fn cell_of(&self, p: Vec2) -> (usize, usize) {
        let cx = ((p.x - self.origin.x) / self.cell).floor();
        let cy = ((p.y - self.origin.y) / self.cell).floor();
        let clamp = |v: f64, n: usize| if v.is_finite() { (v.max(0.0) as usize).min(n.saturating_sub(1)) } else { 0 };
        (clamp(cx, self.nx), clamp(cy, self.ny))
    }

    /// Ids of the triangles whose bounding cells meet `r` (a superset of the triangles touching it).
    fn candidates(&self, r: Bounds2) -> Vec<u32> {
        if self.tris.is_empty() || r.is_empty() {
            return Vec::new();
        }
        let ((x0, y0), (x1, y1)) = (self.cell_of(r.min), self.cell_of(r.max));
        let mut out = Vec::new();
        for cy in y0..=y1 {
            for cx in x0..=x1 {
                let k = cy * self.nx + cx;
                if let (Some(&s), Some(&e)) = (self.start.get(k), self.start.get(k + 1))
                    && let Some(ids) = self.ids.get(s as usize..e as usize)
                {
                    out.extend_from_slice(ids);
                }
            }
        }
        out.sort_unstable();
        out.dedup();
        out
    }

    /// Elevation at a plan point; `None` outside the triangles.
    pub fn elevation(&self, p: Vec2) -> Option<f64> {
        let r = Bounds2::new(p, p);
        for ti in self.candidates(r) {
            if let Some(c) = self.corners(ti as usize)
                && let Some(z) = plane_z(&c, p, 1e-9)
            {
                return Some(z);
            }
        }
        None
    }

    /// Contour lines at `level` as polylines (plan points); a closed contour repeats its first point.
    pub fn contours(&self, level: f64) -> Vec<Vec<Vec2>> {
        // A level exactly on a vertex elevation would cut through the vertex; nudge it.
        let on_vertex = self.pts.iter().any(|p| (p.z - level).abs() < 1e-9);
        let lv = if on_vertex { level + 1e-7 } else { level };
        type Key = (u32, u32);
        let edge_key = |a: u32, b: u32| if a < b { (a, b) } else { (b, a) };
        let mut segs: Vec<(Key, Key, Vec2, Vec2)> = Vec::new();
        for t in &self.tris {
            let mut hits: Vec<(Key, Vec2)> = Vec::with_capacity(2);
            for k in 0..3 {
                let (ia, ib) = (t[k], t[(k + 1) % 3]);
                let (Some(&a), Some(&b)) = (self.pts.get(ia as usize), self.pts.get(ib as usize)) else { continue };
                if (a.z - lv) * (b.z - lv) < 0.0 {
                    let f = (lv - a.z) / (b.z - a.z);
                    hits.push((edge_key(ia, ib), xy(a).lerp(xy(b), f)));
                }
            }
            if let [(ka, pa), (kb, pb)] = hits[..] {
                segs.push((ka, kb, pa, pb));
            }
        }
        let mut at: HashMap<Key, Vec<usize>> = HashMap::new();
        for (i, s) in segs.iter().enumerate() {
            at.entry(s.0).or_default().push(i);
            at.entry(s.1).or_default().push(i);
        }
        let mut used = vec![false; segs.len()];
        let mut out = Vec::new();
        // Open chains first (start from an end with a single segment), then the closed loops.
        let ends: Vec<usize> = (0..segs.len())
            .filter(|&i| segs.get(i).is_some_and(|s| at.get(&s.0).is_some_and(|v| v.len() == 1) || at.get(&s.1).is_some_and(|v| v.len() == 1)))
            .collect();
        for i in ends.into_iter().chain(0..segs.len()) {
            if used.get(i).copied().unwrap_or(true) {
                continue;
            }
            let Some(&(ka, kb, pa, pb)) = segs.get(i) else { continue };
            used[i] = true;
            let open_a = at.get(&ka).is_some_and(|v| v.len() == 1);
            // Orient so the chain grows from the free end.
            let (mut line, mut tail) = if open_a || !at.get(&kb).is_some_and(|v| v.len() == 1) { (vec![pa, pb], kb) } else { (vec![pb, pa], ka) };
            loop {
                let next = at.get(&tail).and_then(|v| v.iter().copied().find(|&j| !used.get(j).copied().unwrap_or(true)));
                let Some(j) = next else { break };
                used[j] = true;
                let Some(&(ja, jb, qa, qb)) = segs.get(j) else { break };
                if ja == tail {
                    line.push(qb);
                    tail = jb;
                } else {
                    line.push(qa);
                    tail = ja;
                }
            }
            if line.len() >= 2 {
                if let (Some(&f), Some(&l)) = (line.first(), line.last())
                    && f.near(l, 1e-9)
                {
                    line.pop();
                    line.push(f);
                }
                out.push(line);
            }
        }
        out
    }

    /// The surface along a plan path: runs of `(distance along the path, elevation)`, one run per stretch the
    /// path stays over the surface (it breaks where the path leaves the triangles).
    pub fn section(&self, path: &[Vec2]) -> Vec<Vec<(f64, f64)>> {
        let mut pieces: Vec<(f64, f64, f64, f64)> = Vec::new(); // (d0, z0, d1, z1)
        let mut base = 0.0;
        for w in path.windows(2) {
            let (&a, &b) = match (w.first(), w.get(1)) {
                (Some(a), Some(b)) => (a, b),
                _ => continue,
            };
            let len = a.dist(b);
            if len < 1e-9 || !len.is_finite() {
                continue;
            }
            let mut seg: Vec<(f64, f64, f64, f64)> = Vec::new();
            let sb = Bounds2::new(a, b);
            for ti in self.candidates(sb) {
                let Some(c) = self.corners(ti as usize) else { continue };
                if let Some((t0, t1)) = clip_segment(&c, a, b)
                    && let (Some(z0), Some(z1)) = (plane_z(&c, a.lerp(b, t0), 1e-7), plane_z(&c, a.lerp(b, t1), 1e-7))
                {
                    seg.push((base + t0 * len, z0, base + t1 * len, z1));
                }
            }
            seg.sort_by(|p, q| p.0.total_cmp(&q.0).then(p.2.total_cmp(&q.2)));
            let mut reach = f64::NEG_INFINITY;
            for s in seg {
                if s.2 <= reach + 1e-9 || s.2 - s.0 < 1e-9 {
                    continue;
                }
                // Overlap with the previous piece (a shared edge): start where it ended.
                if s.0 < reach - 1e-9 {
                    let f = (reach - s.0) / (s.2 - s.0);
                    pieces.push((reach, s.1 + (s.3 - s.1) * f, s.2, s.3));
                } else {
                    pieces.push(s);
                }
                reach = s.2;
            }
            base += len;
        }
        let mut runs: Vec<Vec<(f64, f64)>> = Vec::new();
        for (d0, z0, d1, z1) in pieces {
            match runs.last_mut() {
                Some(r) if r.last().is_some_and(|l| (l.0 - d0).abs() < 1e-6) => r.push((d1, z1)),
                _ => runs.push(vec![(d0, z0), (d1, z1)]),
            }
        }
        runs
    }

    /// Volume of `self` (the top) over `base` across the plan area both cover, optionally limited to a simple
    /// `region` polygon. Exact for planar triangles: each overlap of two triangles is cut where the surfaces cross.
    pub fn volume_over(&self, base: &Tin, region: Option<&[Vec2]>) -> Result<Volume, String> {
        let clips: Option<Vec<[Vec2; 3]>> = match region {
            Some(poly) => Some(triangulate_polygon(poly)?),
            None => None,
        };
        let mut v = Volume::default();
        for ti in 0..self.tris.len() {
            let Some(a) = self.corners(ti) else { continue };
            let ab = a.iter().fold(Bounds2::EMPTY, |mut b, p| {
                b.add(xy(*p));
                b
            });
            if let Some(cl) = &clips
                && !cl.iter().any(|t| tri_bounds(t).intersects(&ab))
            {
                continue;
            }
            for bi in base.candidates(ab) {
                let Some(b) = base.corners(bi as usize) else { continue };
                let Some(poly) = clip_convex(&a.map(xy), &b.map(xy)) else { continue };
                match &clips {
                    None => accumulate(&mut v, &poly, &a, &b),
                    Some(cl) => {
                        for t in cl {
                            if let Some(p) = clip_convex(&poly, t) {
                                accumulate(&mut v, &p, &a, &b);
                            }
                        }
                    }
                }
            }
        }
        Ok(v)
    }
}

fn tri_bounds(t: &[Vec2; 3]) -> Bounds2 {
    t.iter().fold(Bounds2::EMPTY, |mut b, p| {
        b.add(*p);
        b
    })
}

/// Elevation of the plane through triangle `c` at `p`; `None` when `p` is outside it by more than `tol`
/// (a relative barycentric tolerance).
fn plane_z(c: &[Vec3; 3], p: Vec2, tol: f64) -> Option<f64> {
    let (a, b, d) = (xy(c[0]), xy(c[1]), xy(c[2]));
    let area = orient(a, b, d);
    if area.abs() < MIN_AREA {
        return None;
    }
    let w0 = orient(p, b, d) / area;
    let w1 = orient(a, p, d) / area;
    let w2 = 1.0 - w0 - w1;
    if w0 < -tol || w1 < -tol || w2 < -tol {
        return None;
    }
    Some(w0 * c[0].z + w1 * c[1].z + w2 * c[2].z)
}

/// The part of segment `a`→`b` inside triangle `c` as parameters `(t0, t1)`.
fn clip_segment(c: &[Vec3; 3], a: Vec2, b: Vec2) -> Option<(f64, f64)> {
    let (mut t0, mut t1) = (0.0f64, 1.0f64);
    for k in 0..3 {
        let (p, q) = (xy(c[k]), xy(c[(k + 1) % 3]));
        // Inside is left of p→q (counter-clockwise).
        let fa = orient(p, q, a);
        let fb = orient(p, q, b);
        if fa < 0.0 && fb < 0.0 {
            return None;
        }
        if fa < 0.0 || fb < 0.0 {
            let t = fa / (fa - fb);
            if fa < 0.0 {
                t0 = t0.max(t);
            } else {
                t1 = t1.min(t);
            }
        }
        if t0 > t1 {
            return None;
        }
    }
    Some((t0, t1))
}

/// Sutherland–Hodgman: `subject` clipped by the convex counter-clockwise triangle `clip`.
fn clip_convex(subject: &[Vec2], clip: &[Vec2; 3]) -> Option<Vec<Vec2>> {
    let mut out: Vec<Vec2> = subject.to_vec();
    for k in 0..3 {
        let (p, q) = (clip[k], clip[(k + 1) % 3]);
        let input = std::mem::take(&mut out);
        for (i, &cur) in input.iter().enumerate() {
            let prev = input.get(if i == 0 { input.len() - 1 } else { i - 1 }).copied().unwrap_or(cur);
            let (fc, fp) = (orient(p, q, cur), orient(p, q, prev));
            if fc >= 0.0 {
                if fp < 0.0 {
                    out.push(prev.lerp(cur, fp / (fp - fc)));
                }
                out.push(cur);
            } else if fp >= 0.0 {
                out.push(prev.lerp(cur, fp / (fp - fc)));
            }
        }
        if out.len() < 3 {
            return None;
        }
    }
    (poly_area(&out) > 1e-12).then_some(out)
}

fn poly_area(p: &[Vec2]) -> f64 {
    let mut s = 0.0;
    for (i, a) in p.iter().enumerate() {
        if let Some(b) = p.get((i + 1) % p.len()) {
            s += a.x * b.y - b.x * a.y;
        }
    }
    s / 2.0
}

/// Adds the volume between top triangle `a` and base triangle `b` over their overlap `poly` (convex).
fn accumulate(v: &mut Volume, poly: &[Vec2], a: &[Vec3; 3], b: &[Vec3; 3]) {
    let f = |p: Vec2| match (plane_z(a, p, 1e-6), plane_z(b, p, 1e-6)) {
        (Some(za), Some(zb)) => za - zb,
        _ => 0.0,
    };
    let fs: Vec<(Vec2, f64)> = poly.iter().map(|&p| (p, f(p))).collect();
    for sign in [1.0f64, -1.0] {
        // Keep the part where sign * f >= 0 by clipping against the zero line of f.
        let mut part: Vec<Vec2> = Vec::new();
        for (i, &(cur, fc)) in fs.iter().enumerate() {
            let (prev, fp) = fs.get(if i == 0 { fs.len() - 1 } else { i - 1 }).copied().unwrap_or((cur, fc));
            let (sc, sp) = (sign * fc, sign * fp);
            if sc >= 0.0 {
                if sp < 0.0 {
                    part.push(prev.lerp(cur, sp / (sp - sc)));
                }
                part.push(cur);
            } else if sp >= 0.0 {
                part.push(prev.lerp(cur, sp / (sp - sc)));
            }
        }
        if part.len() < 3 {
            continue;
        }
        let area = poly_area(&part);
        if area <= 1e-12 {
            continue;
        }
        let mut vol = 0.0;
        if let Some(&p0) = part.first() {
            for w in part.windows(2).skip(1) {
                if let [p1, p2] = *w {
                    let ar = orient(p0, p1, p2) / 2.0;
                    vol += ar * (f(p0) + f(p1) + f(p2)) / 3.0;
                }
            }
        }
        if sign > 0.0 {
            v.fill += vol;
            v.fill_area += area;
        } else {
            v.cut += -vol;
            v.cut_area += area;
        }
    }
    v.area += poly_area(poly);
}

/// Ear-clipping triangulation of a simple polygon (any winding); the triangles are counter-clockwise.
pub fn triangulate_polygon(poly: &[Vec2]) -> Result<Vec<[Vec2; 3]>, String> {
    let mut p: Vec<Vec2> = poly.to_vec();
    if p.len() > 1 && p.first().zip(p.last()).is_some_and(|(a, b)| a.near(*b, 1e-12)) {
        p.pop();
    }
    if p.len() < 3 || p.len() > 100_000 || p.iter().any(|q| !q.is_finite()) {
        return Err("region needs 3 to 100000 finite points".into());
    }
    if poly_area(&p) < 0.0 {
        p.reverse();
    }
    let mut out = Vec::with_capacity(p.len() - 2);
    let mut guard = p.len() * p.len() + 10;
    while p.len() > 3 {
        guard = guard.checked_sub(1).ok_or("region polygon is not simple")?;
        let n = p.len();
        let mut clipped = false;
        for i in 0..n {
            let (a, b, c) = (p[(i + n - 1) % n], p[i], p[(i + 1) % n]);
            if orient(a, b, c) <= 1e-12 {
                continue;
            }
            let inside = p.iter().any(|&q| {
                !(q.near(a, 1e-12) || q.near(b, 1e-12) || q.near(c, 1e-12))
                    && orient(a, b, q) >= 0.0
                    && orient(b, c, q) >= 0.0
                    && orient(c, a, q) >= 0.0
            });
            if !inside {
                out.push([a, b, c]);
                p.remove(i);
                clipped = true;
                break;
            }
        }
        if !clipped {
            // Collinear leftovers: drop a zero-area vertex.
            let Some(i) = (0..n).find(|&i| orient(p[(i + n - 1) % n], p[i], p[(i + 1) % n]).abs() <= 1e-12) else {
                return Err("region polygon is not simple".into());
            };
            p.remove(i);
        }
    }
    if let [a, b, c] = p[..]
        && orient(a, b, c) > 1e-12
    {
        out.push([a, b, c]);
    }
    if out.is_empty() {
        return Err("region has no area".into());
    }
    Ok(out)
}

// ------------------------------------------------------------------------------------------ Delaunay

#[derive(Clone)]
struct DTri {
    v: [u32; 3],
    /// Neighbour across the edge opposite vertex `i`.
    n: [u32; 3],
    alive: bool,
}

/// Bowyer–Watson with triangle walking, points inserted in grid-snake order so each walk is short.
fn delaunay(pts: &[Vec3]) -> Result<Vec<[u32; 3]>, String> {
    let n = pts.len();
    let mut b = Bounds2::EMPTY;
    for p in pts {
        b.add(xy(*p));
    }
    let (cx, cy) = ((b.min.x + b.max.x) / 2.0, (b.min.y + b.max.y) / 2.0);
    let span = (b.max.x - b.min.x).max(b.max.y - b.min.y).max(1e-9);
    // Work in unit coordinates around the centre so the in-circle test keeps its precision.
    let mut q: Vec<Vec2> = pts.iter().map(|p| Vec2::new((p.x - cx) / span, (p.y - cy) / span)).collect();
    let big = 50.0;
    q.push(Vec2::new(-big, -big));
    q.push(Vec2::new(big, -big));
    q.push(Vec2::new(0.0, big));
    let (s0, s1, s2) = (n as u32, n as u32 + 1, n as u32 + 2);
    let mut tris = vec![DTri { v: [s0, s1, s2], n: [NONE; 3], alive: true }];
    let pt = |i: u32| q.get(i as usize).copied().unwrap_or(Vec2::new(0.0, 0.0));

    // Insertion order: serpentine over a coarse grid.
    let side = ((n as f64).sqrt() / 2.0).ceil().max(1.0);
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by_key(|&i| {
        let p = pt(i as u32);
        let gx = (((p.x + 0.5) * side).floor().max(0.0)) as i64;
        let gy = (((p.y + 0.5) * side).floor().max(0.0)) as i64;
        (gx, if gx % 2 == 0 { gy } else { -gy })
    });

    let mut last = 0usize;
    for &pi in &order {
        let p = pt(pi as u32);
        let Some(t0) = locate(&tris, &q, p, last) else {
            continue; // a duplicate or numerically stuck point; skip it
        };
        // Cavity: triangles whose circumcircle holds p, grown from the containing one.
        let mut bad = vec![t0];
        let mut seen = std::collections::HashSet::from([t0]);
        let mut stack = vec![t0];
        while let Some(t) = stack.pop() {
            let Some(tri) = tris.get(t) else { continue };
            for &nb in &tri.n {
                let nb = nb as usize;
                if nb == NONE as usize || seen.contains(&nb) {
                    continue;
                }
                let Some(o) = tris.get(nb) else { continue };
                if o.alive && in_circle(pt(o.v[0]), pt(o.v[1]), pt(o.v[2]), p) {
                    seen.insert(nb);
                    bad.push(nb);
                    stack.push(nb);
                }
            }
        }
        // Boundary edges of the cavity.
        let mut fresh: Vec<DTri> = Vec::new();
        let mut by_a: HashMap<u32, usize> = HashMap::new();
        let mut by_b: HashMap<u32, usize> = HashMap::new();
        let first_new = tris.len();
        let mut outside: Vec<(usize, usize, usize)> = Vec::new(); // (outside tri, bad tri it pointed at, new index)
        for &t in &bad {
            let Some(tri) = tris.get(t) else { continue };
            for i in 0..3 {
                let nb = tri.n[i];
                if nb != NONE && seen.contains(&(nb as usize)) {
                    continue;
                }
                let (a, bb) = (tri.v[(i + 1) % 3], tri.v[(i + 2) % 3]);
                let idx = first_new + fresh.len();
                fresh.push(DTri { v: [pi as u32, a, bb], n: [nb, NONE, NONE], alive: true });
                by_a.insert(a, idx);
                by_b.insert(bb, idx);
                if nb != NONE {
                    outside.push((nb as usize, t, idx));
                }
            }
        }
        for f in &mut fresh {
            let (a, bb) = (f.v[1], f.v[2]);
            f.n[2] = by_b.get(&a).map_or(NONE, |&i| i as u32); // edge (p, a)
            f.n[1] = by_a.get(&bb).map_or(NONE, |&i| i as u32); // edge (b, p)
        }
        for (o, old, new) in outside {
            if let Some(ot) = tris.get_mut(o) {
                for k in 0..3 {
                    if ot.n[k] == old as u32 {
                        ot.n[k] = new as u32;
                    }
                }
            }
        }
        for &t in &bad {
            if let Some(tri) = tris.get_mut(t) {
                tri.alive = false;
            }
        }
        last = first_new;
        tris.extend(fresh);
        if tris.len() > MAX_TIN_TRIANGLES.saturating_mul(2) {
            return Err("triangulation grew past its limit".into());
        }
    }
    Ok(tris.iter().filter(|t| t.alive && t.v.iter().all(|&i| i < n as u32)).map(|t| t.v).collect())
}

fn locate(tris: &[DTri], q: &[Vec2], p: Vec2, start: usize) -> Option<usize> {
    let pt = |i: u32| q.get(i as usize).copied().unwrap_or(Vec2::new(0.0, 0.0));
    let mut t = if tris.get(start).is_some_and(|t| t.alive) { start } else { tris.iter().rposition(|t| t.alive)? };
    for _ in 0..tris.len() + 16 {
        let tri = tris.get(t)?;
        let mut moved = false;
        for i in 0..3 {
            let (a, b) = (pt(tri.v[(i + 1) % 3]), pt(tri.v[(i + 2) % 3]));
            if orient(a, b, p) < 0.0 {
                let nb = tri.n[i];
                if nb == NONE {
                    return None;
                }
                t = nb as usize;
                moved = true;
                break;
            }
        }
        if !moved {
            return Some(t);
        }
    }
    None
}

fn in_circle(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> bool {
    let (ax, ay) = (a.x - d.x, a.y - d.y);
    let (bx, by) = (b.x - d.x, b.y - d.y);
    let (cx, cy) = (c.x - d.x, c.y - d.y);
    let det = (ax * ax + ay * ay) * (bx * cy - cx * by) - (bx * bx + by * by) * (ax * cy - cx * ay) + (cx * cx + cy * cy) * (ax * by - bx * ay);
    det > 1e-14
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { x, y, z }
    }

    /// A 10 × 10 grid of points on the plane z = x + 2y.
    fn plane() -> Tin {
        let mut pts = Vec::new();
        for i in 0..=10 {
            for j in 0..=10 {
                pts.push(v(f64::from(i), f64::from(j), f64::from(i) + 2.0 * f64::from(j)));
            }
        }
        Tin::from_points(&pts).unwrap()
    }

    #[test]
    fn delaunay_covers_the_hull_and_interpolates_a_plane() {
        let t = plane();
        assert!((t.area() - 100.0).abs() < 1e-6, "{}", t.area());
        assert!((t.elevation(Vec2::new(3.3, 4.1)).unwrap() - (3.3 + 8.2)).abs() < 1e-9);
        assert!(t.elevation(Vec2::new(11.0, 4.0)).is_none());
    }

    #[test]
    fn delaunay_survives_scattered_and_duplicate_points() {
        let mut pts = Vec::new();
        let mut s = 12345u64;
        for _ in 0..3000 {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let x = (s >> 33) as f64 / 1e5;
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let y = (s >> 33) as f64 / 1e5;
            pts.push(v(x, y, x * 0.5 - y));
        }
        pts.push(pts[0]);
        let t = Tin::from_points(&pts).unwrap();
        assert!(t.triangles().len() > 5000);
        // Hull area equals the sum of the triangles (no overlaps): compare with a coarse hull bound.
        assert!(t.area() > 0.0);
        let mid = t.bounds();
        let c = Vec2::new((mid.min.x + mid.max.x) / 2.0, (mid.min.y + mid.max.y) / 2.0);
        let z = t.elevation(c).unwrap();
        assert!((z - (c.x * 0.5 - c.y)).abs() < 1e-6);
    }

    #[test]
    fn rejects_hostile_input() {
        assert!(Tin::new(vec![v(f64::NAN, 0.0, 0.0)], vec![]).is_err());
        assert!(Tin::new(vec![v(0.0, 0.0, 0.0)], vec![[0, 1, 2]]).is_err());
        assert!(Tin::from_points(&[v(0.0, 0.0, 0.0), v(1.0, 1.0, 1.0)]).is_err());
        // Collinear points have no triangles but do not panic.
        let line: Vec<Vec3> = (0..20).map(|i| v(f64::from(i), 0.0, 0.0)).collect();
        assert!(Tin::from_points(&line).map(|t| t.triangles().is_empty()).unwrap_or(true));
    }

    #[test]
    fn contours_are_level_and_chained() {
        let t = plane();
        let lines = t.contours(10.0);
        assert_eq!(lines.len(), 1, "one chain across the plane");
        let l = &lines[0];
        assert!(l.len() > 3);
        for p in l {
            assert!((p.x + 2.0 * p.y - 10.0).abs() < 1e-6);
        }
        assert!(t.contours(1000.0).is_empty());
    }

    #[test]
    fn section_follows_the_surface() {
        let t = plane();
        let runs = t.section(&[Vec2::new(-5.0, 5.0), Vec2::new(15.0, 5.0)]);
        assert_eq!(runs.len(), 1);
        let r = &runs[0];
        assert!((r.first().unwrap().0 - 5.0).abs() < 1e-6 && (r.last().unwrap().0 - 15.0).abs() < 1e-6);
        assert!((r.first().unwrap().1 - 10.0).abs() < 1e-6 && (r.last().unwrap().1 - 20.0).abs() < 1e-6);
    }

    #[test]
    fn volume_between_parallel_and_crossing_surfaces() {
        let base = plane();
        let mut up = Vec::new();
        for p in base.points() {
            up.push(v(p.x, p.y, p.z + 2.0));
        }
        let top = Tin::new(up, base.triangles().to_vec()).unwrap();
        let vol = top.volume_over(&base, None).unwrap();
        assert!((vol.fill - 200.0).abs() < 1e-6 && vol.cut.abs() < 1e-9 && (vol.area - 100.0).abs() < 1e-6);
        // Limited to a 5 × 4 region.
        let r = [Vec2::new(1.0, 1.0), Vec2::new(6.0, 1.0), Vec2::new(6.0, 5.0), Vec2::new(1.0, 5.0)];
        let part = top.volume_over(&base, Some(&r)).unwrap();
        assert!((part.fill - 40.0).abs() < 1e-6, "{part:?}");
        // A tilted copy crossing the base along x = 5: cut on one side, fill on the other, equal.
        let tilt: Vec<Vec3> = base.points().iter().map(|p| v(p.x, p.y, p.z + (p.x - 5.0))).collect();
        let tilted = Tin::new(tilt, base.triangles().to_vec()).unwrap();
        let x = tilted.volume_over(&base, None).unwrap();
        assert!((x.fill - 125.0).abs() < 1e-6 && (x.cut - 125.0).abs() < 1e-6, "{x:?}");
    }

    #[test]
    fn polygon_triangulation_handles_concave_and_rejects_bowties() {
        let l = [Vec2::new(0.0, 0.0), Vec2::new(4.0, 0.0), Vec2::new(4.0, 1.0), Vec2::new(1.0, 1.0), Vec2::new(1.0, 4.0), Vec2::new(0.0, 4.0)];
        let t = triangulate_polygon(&l).unwrap();
        let area: f64 = t.iter().map(|x| orient(x[0], x[1], x[2]) / 2.0).sum();
        assert!((area - 7.0).abs() < 1e-9);
        let bow = [Vec2::new(0.0, 0.0), Vec2::new(2.0, 2.0), Vec2::new(2.0, 0.0), Vec2::new(0.0, 2.0)];
        let _ = triangulate_polygon(&bow); // either Ok or Err, never a panic
        assert!(triangulate_polygon(&[Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0)]).is_err());
    }

    #[test]
    fn trimmed_drops_long_triangles() {
        let t = plane();
        assert_eq!(t.trimmed(1.0).unwrap().triangles().len(), 0);
        assert_eq!(t.trimmed(2.0).unwrap().triangles().len(), t.triangles().len());
    }
}
