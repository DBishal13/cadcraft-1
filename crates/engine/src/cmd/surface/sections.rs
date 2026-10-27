//! SURFACESECTION (a profile of surfaces along any path) and SURFACESECTIONS (cross sections at stations of
//! an alignment, with the end areas between the first two surfaces).

use serde_json::{Value, json};

use cadcraft_geom::Vec2;

use super::{Drawn, path_param, surface};
use crate::cmd::{bad, f64_or, point_req, str_param};
use crate::{Result, Session};

type Runs = Vec<Vec<(f64, f64)>>;

/// Layer colours cycled through for the surfaces, in the order they were named.
const PALETTE: [u8; 6] = [3, 1, 4, 2, 6, 30];

/// A step of 1, 2 or 5 times a power of ten near `raw`.
fn nice(raw: f64) -> f64 {
    if !(raw.is_finite() && raw > 0.0) {
        return 1.0;
    }
    let p = 10f64.powf(raw.log10().floor());
    let f = raw / p;
    p * if f <= 1.0 {
        1.0
    } else if f <= 2.0 {
        2.0
    } else if f <= 5.0 {
        5.0
    } else {
        10.0
    }
}

struct Plot {
    origin: Vec2,
    hscale: f64,
    vscale: f64,
    d0: f64,
    d1: f64,
    zlo: f64,
    zhi: f64,
    lh: f64,
    /// Added to a distance for its label (a cross section's offset from the centre line).
    shift: f64,
    title: String,
}

impl Plot {
    fn pt(&self, d: f64, z: f64) -> Vec2 {
        Vec2::new(self.origin.x + (d - self.d0) * self.hscale, self.origin.y + (z - self.zlo) * self.vscale)
    }
}

fn draw_plot(s: &mut Session, d: &mut Drawn, pl: &Plot, series: &[(&str, u8, &Runs)]) -> Result<()> {
    let (a, b) = (pl.pt(pl.d0, pl.zlo), pl.pt(pl.d1, pl.zhi));
    let frame = "SEC-FRAME";
    for (p, q) in [(a, Vec2::new(b.x, a.y)), (Vec2::new(b.x, a.y), b), (b, Vec2::new(a.x, b.y)), (Vec2::new(a.x, b.y), a)] {
        d.line(s, frame, 7, p, q)?;
    }
    let zstep = nice((pl.zhi - pl.zlo) / 6.0);
    let mut z = (pl.zlo / zstep).ceil() * zstep;
    let mut n = 0;
    while z <= pl.zhi + 1e-9 && n < 200 {
        n += 1;
        let (l, r) = (pl.pt(pl.d0, z), pl.pt(pl.d1, z));
        d.line(s, "SEC-GRID", 8, l, r)?;
        d.text(s, "SEC-TEXT", 7, Vec2::new(l.x - pl.lh * 2.5, l.y), pl.lh, 0.0, &format!("{z:.0}"))?;
        z += zstep;
    }
    let dstep = nice((pl.d1 - pl.d0) / 6.0);
    let mut x = (pl.d0 / dstep).ceil() * dstep;
    n = 0;
    while x <= pl.d1 + 1e-9 && n < 200 {
        n += 1;
        let (l, t) = (pl.pt(x, pl.zlo), pl.pt(x, pl.zhi));
        d.line(s, "SEC-GRID", 8, l, t)?;
        d.text(s, "SEC-TEXT", 7, Vec2::new(l.x, l.y - pl.lh * 1.5), pl.lh, 0.0, &format!("{:.0}", x + pl.shift))?;
        x += dstep;
    }
    d.text(s, "SEC-TEXT", 7, Vec2::new((a.x + b.x) / 2.0, b.y + pl.lh * 2.0), pl.lh * 1.4, 0.0, &pl.title)?;
    for (name, color, runs) in series {
        let layer = format!("SEC-{name}");
        for run in runs.iter() {
            let pts: Vec<Vec2> = run.iter().map(|(dd, zz)| pl.pt(*dd, *zz)).collect();
            if pts.len() >= 2 {
                d.polyline(s, &layer, *color, &pts, 0.0)?;
            }
        }
    }
    Ok(())
}

fn names(p: &Value, cmd: &str) -> Result<Vec<String>> {
    let v: Vec<String> =
        p.get("surfaces").and_then(Value::as_array).map(|a| a.iter().filter_map(|x| x.as_str()).map(str::to_string).collect()).unwrap_or_default();
    if v.is_empty() {
        return Err(bad(cmd, "`surfaces` is a list of loaded surface names"));
    }
    Ok(v)
}

fn z_extent(all: &[Vec<Runs>]) -> Option<(f64, f64)> {
    let mut r: Option<(f64, f64)> = None;
    for runs in all.iter().flatten().flatten().flatten() {
        r = Some(match r {
            None => (runs.1, runs.1),
            Some((lo, hi)) => (lo.min(runs.1), hi.max(runs.1)),
        });
    }
    r
}

pub(super) fn run_section(s: &mut Session, p: &Value) -> Result<Value> {
    let path = path_param(p, "path", "surfacesection")?;
    let names = names(p, "surfacesection")?;
    let at = point_req("surfacesection", p, "at")?;
    let (hs, vs) = (f64_or(p, "hscale", 1.0), f64_or(p, "vscale", 10.0));
    if hs <= 0.0 || vs <= 0.0 {
        return Err(bad("surfacesection", "`hscale` and `vscale` must be positive"));
    }
    let mut all: Vec<Runs> = Vec::new();
    for n in &names {
        all.push(surface(s, n)?.section(&path.pts));
    }
    let all = [all];
    let (zmin, zmax) = z_extent(&all).ok_or_else(|| bad("surfacesection", "the path does not cross any of the surfaces"))?;
    let step = nice((zmax - zmin) / 6.0);
    let zlo = p.get("datum").and_then(Value::as_f64).unwrap_or_else(|| (zmin / step).floor() * step - step);
    let zhi = ((zmax / step).ceil() * step + step).max(zlo + step);
    let lh = f64_or(p, "labelHeight", (path.length() * hs / 80.0).max(1e-6));
    let plot = Plot {
        origin: at,
        hscale: hs,
        vscale: vs,
        d0: 0.0,
        d1: path.length(),
        zlo,
        zhi,
        lh,
        shift: 0.0,
        title: str_param(p, "title").unwrap_or("PROFILE").to_string(),
    };
    let mut d = Drawn::begin(s)?;
    let series: Vec<(&str, u8, &Runs)> =
        names.iter().zip(all.first().into_iter().flatten()).enumerate().map(|(i, (n, r))| (n.as_str(), PALETTE[i % PALETTE.len()], r)).collect();
    let res = draw_plot(s, &mut d, &plot, &series);
    let drawn = d.count;
    d.end(s)?;
    res?;
    let summary: Vec<Value> = series.iter().map(|(n, _, r)| json!({ "name": n, "runs": r.len() })).collect();
    Ok(json!({ "surfaces": summary, "objects": drawn, "size": [path.length() * hs, (zhi - zlo) * vs] }))
}

/// `runs` evaluated at distance `d` (`None` between runs and outside them).
fn eval(runs: &Runs, d: f64) -> Option<f64> {
    for r in runs {
        for w in r.windows(2) {
            if let [(d0, z0), (d1, z1)] = w[..]
                && d >= d0 - 1e-9
                && d <= d1 + 1e-9
            {
                return Some(if d1 > d0 { z0 + (z1 - z0) * ((d - d0) / (d1 - d0)).clamp(0.0, 1.0) } else { z0 });
            }
        }
    }
    None
}

/// Area between the top and base profiles where both exist: `(fill, cut)`.
fn end_area(top: &Runs, base: &Runs) -> (f64, f64) {
    let mut ds: Vec<f64> = top.iter().chain(base.iter()).flatten().map(|p| p.0).collect();
    ds.sort_by(f64::total_cmp);
    ds.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    let (mut fill, mut cut) = (0.0, 0.0);
    for w in ds.windows(2) {
        let (&a, &b) = match (w.first(), w.get(1)) {
            (Some(a), Some(b)) => (a, b),
            _ => continue,
        };
        let mid = (a + b) / 2.0;
        if eval(top, mid).is_none() || eval(base, mid).is_none() {
            continue;
        }
        let diff = |x: f64| match (eval(top, x), eval(base, x)) {
            (Some(t), Some(bb)) => t - bb,
            _ => 0.0,
        };
        let (fa, fb) = (diff(a), diff(b));
        let w = b - a;
        if fa >= 0.0 && fb >= 0.0 {
            fill += (fa + fb) / 2.0 * w;
        } else if fa <= 0.0 && fb <= 0.0 {
            cut += -(fa + fb) / 2.0 * w;
        } else {
            let t = fa / (fa - fb); // where the profiles cross
            let (pos, neg) = if fa > 0.0 { (fa * t * w / 2.0, -fb * (1.0 - t) * w / 2.0) } else { (fb * (1.0 - t) * w / 2.0, -fa * t * w / 2.0) };
            fill += pos;
            cut += neg;
        }
    }
    (fill, cut)
}

pub(super) fn run_sections(s: &mut Session, p: &Value) -> Result<Value> {
    let path = path_param(p, "alignment", "surfacesections")?;
    let names = names(p, "surfacesections")?;
    let surfaces: Vec<_> = names.iter().map(|n| surface(s, n)).collect::<Result<_>>()?;
    let at = point_req("surfacesections", p, "at")?;
    let start = f64_or(p, "startStation", 0.0);
    let interval = f64_or(p, "interval", 100.0);
    let width = f64_or(p, "width", 125.0);
    let (hs, vs) = (f64_or(p, "hscale", 1.0), f64_or(p, "vscale", 10.0));
    let columns = p.get("columns").and_then(Value::as_u64).unwrap_or(4).clamp(1, 50) as usize;
    let divisor = f64_or(p, "divisor", 1.0);
    if interval <= 0.0 || width <= 0.0 || hs <= 0.0 || vs <= 0.0 || divisor <= 0.0 {
        return Err(bad("surfacesections", "`interval`, `width`, `hscale`, `vscale` and `divisor` must be positive"));
    }
    let (from, to) = (f64_or(p, "from", start), f64_or(p, "to", start + path.length()));
    let mut stations = Vec::new();
    let mut st = (from / interval).ceil() * interval;
    while st <= to + 1e-9 && stations.len() < 2000 {
        if st >= start - 1e-9 && st - start <= path.length() + 1e-9 {
            stations.push(st);
        }
        st += interval;
    }
    if stations.is_empty() {
        return Err(bad("surfacesections", "no station of that interval lies on the alignment"));
    }
    // A section crosses the alignment, left end first.
    let mut sections: Vec<Vec<Runs>> = Vec::new();
    for st in &stations {
        let (c, t) = path.at(st - start);
        let n = Vec2::new(-t.y, t.x);
        let line = [Vec2::new(c.x + n.x * width, c.y + n.y * width), Vec2::new(c.x - n.x * width, c.y - n.y * width)];
        sections.push(surfaces.iter().map(|t| t.section(&line)).collect());
    }
    let (zmin, zmax) = z_extent(&sections).ok_or_else(|| bad("surfacesections", "the sections do not cross any of the surfaces"))?;
    let step = nice((zmax - zmin) / 6.0);
    let (zlo, zhi) = ((zmin / step).floor() * step - step, (zmax / step).ceil() * step + step);
    let lh = f64_or(p, "labelHeight", (width * hs / 30.0).max(1e-6));
    let (cell_w, cell_h) = (2.0 * width * hs + lh * 12.0, (zhi - zlo) * vs + lh * 8.0);
    let mut d = Drawn::begin(s)?;
    let mut rows = Vec::new();
    let mut areas: Vec<(f64, f64, f64)> = Vec::new();
    let res = (|| -> Result<()> {
        for (i, (st, runs)) in stations.iter().zip(&sections).enumerate() {
            let (col, row) = (i % columns, i / columns);
            let origin = Vec2::new(at.x + col as f64 * cell_w + lh * 6.0, at.y - (row as f64 + 1.0) * cell_h + lh * 4.0);
            let plot = Plot {
                origin,
                hscale: hs,
                vscale: vs,
                d0: 0.0,
                d1: 2.0 * width,
                zlo,
                zhi,
                lh,
                shift: -width,
                title: format!("STA {}", fmt_sta(*st)),
            };
            let series: Vec<(&str, u8, &Runs)> =
                names.iter().zip(runs).enumerate().map(|(k, (n, r))| (n.as_str(), PALETTE[k % PALETTE.len()], r)).collect();
            draw_plot(s, &mut d, &plot, &series)?;
            if let [top, base, ..] = &runs[..] {
                let (f, c) = end_area(top, base);
                d.text(s, "SEC-TEXT", 7, Vec2::new(origin.x + width * hs, origin.y - lh * 3.5), lh, 0.0, &format!("FILL {f:.1}   CUT {c:.1}"))?;
                areas.push((*st, f, c));
            }
            rows.push(json!({ "station": st, "fillArea": areas.last().filter(|a| a.0 == *st).map(|a| a.1), "cutArea": areas.last().filter(|a| a.0 == *st).map(|a| a.2) }));
        }
        Ok(())
    })();
    let drawn = d.count;
    d.end(s)?;
    res?;
    // Average end area between neighbouring sections.
    let (mut fill, mut cut) = (0.0, 0.0);
    for w in areas.windows(2) {
        if let [a, b] = w {
            let len = b.0 - a.0;
            fill += (a.1 + b.1) / 2.0 * len / divisor;
            cut += (a.2 + b.2) / 2.0 * len / divisor;
        }
    }
    let volume = (areas.len() >= 2).then(|| json!({ "fill": fill, "cut": cut, "net": fill - cut }));
    Ok(json!({ "sections": rows, "count": stations.len(), "objects": drawn, "volume": volume }))
}

fn fmt_sta(v: f64) -> String {
    let whole = (v / 100.0).floor();
    format!("{}+{:02.0}", whole as i64, v - whole * 100.0)
}
