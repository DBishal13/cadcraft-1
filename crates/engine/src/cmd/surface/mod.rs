//! Terrain surfaces (TINs): LANDXMLIN and SURFACECREATE load them into the session; SURFACECONTOURS,
//! SURFACETIN, SURFACESECTION and SURFACESECTIONS draw them; SURFACEVOLUME and SURFACESTRIPS measure the
//! volume between two. Surfaces live in the session (reload a LandXML file to bring them back), not in the DWG.

mod landxml;
mod sections;

use std::sync::Arc;

use cadcraft_doc::{EntityKind, HAlign, Layer, LwPolyline, Text, VAlign};
use cadcraft_geom::tin::Tin;
use cadcraft_geom::{Bounds2, PolyVertex, Polyline, Vec2, Vec3};
use serde_json::{Value, json};

use super::helpers::{line, v3};
use super::{CommandSpec, always, bad, bool_or, f64_or, point_value, points_param, str_param};
use crate::{EngineError, Result, Session};

/// Most entities one drawing command may add.
pub(crate) const MAX_DRAWN: usize = 400_000;
/// Largest LandXML file read.
#[cfg(not(target_arch = "wasm32"))]
const MAX_XML_BYTES: usize = 1 << 30;

pub fn specs() -> Vec<CommandSpec> {
    vec![
        CommandSpec::new("landxmlin", "Import LandXML Surfaces", run_landxmlin)
            .enabled(always)
            .noundo()
            .params("{path, names?: [substring,...] (only surfaces whose name holds one), maxEdge?: drop triangles with a longer plan edge} → {surfaces: [{name, points, triangles, zMin, zMax, area}]}"),
        CommandSpec::new("surface.create", "Create Surface From Points", run_create)
            .enabled(always)
            .noundo()
            .params("{name, points: [[x,y,z],...], maxEdge?} → {name, points, triangles}"),
        CommandSpec::new("surface.list", "List Surfaces", run_list).enabled(always).noundo().params("{} → {surfaces: [{name, points, triangles, zMin, zMax, area}]}"),
        CommandSpec::new("surface.delete", "Delete Surface", run_delete).enabled(always).noundo().params("{name | all: true}"),
        CommandSpec::new("surfacecontours", "Draw Surface Contours", run_contours)
            .params("{surface, interval, majorEvery? (default 5), window?: [[x,y],[x,y]], from?, to?, labels?: bool (default true), labelHeight?, minorLayer? (CONTOUR-MINOR), majorLayer? (CONTOUR-MAJOR)} → {levels, polylines, vertices}"),
        CommandSpec::new("surfacetin", "Draw Surface Triangles", run_tin)
            .params("{surface, window?: [[x,y],[x,y]], layer? (TIN)} → {edges}"),
        CommandSpec::new("surfacesection", "Draw Surface Profile", sections::run_section)
            .params("{path: [[x,y],...], surfaces: [name,...], at: [x,y] (lower-left of the graph), hscale? (1), vscale? (10), datum?, labelHeight?, title?} → {surfaces: [{name, runs}], size}"),
        CommandSpec::new("surfacesections", "Draw Cross Sections", sections::run_sections)
            .params("{alignment: [[x,y],...], surfaces: [name,...] (the first two are compared), startStation? (0), from?, to?, interval? (100), width? (125, each side), at: [x,y], columns? (4), hscale? (1), vscale? (10), labelHeight?} → {sections: [{station, fillArea, cutArea}], count, volume?: {fill, cut}}"),
        CommandSpec::new("surfacevolume", "Volume Between Surfaces", run_volume)
            .enabled(always)
            .noundo()
            .params("{top, base, region?: [[x,y],...] (a simple polygon)} → {fill, cut, net, area}"),
        CommandSpec::new("surfacestrips", "Volume By Station Strips", run_strips)
            .enabled(always)
            .noundo()
            .params("{top, base, alignment: [[x,y],...], startStation? (0), from?, to?, step? (100), width? (125), side? (1 left, -1 right, 0 both)} → {strips: [{s0, s1, fill, cut, net}], fill, cut, net}"),
    ]
}

// ------------------------------------------------------------------------------------------ registry

pub(crate) fn surface(s: &Session, name: &str) -> Result<Arc<Tin>> {
    s.surfaces.get(name).cloned().ok_or_else(|| {
        let known: Vec<&str> = s.surfaces.keys().map(String::as_str).collect();
        bad("surface", format!("no surface `{name}` (loaded: {})", if known.is_empty() { "none".into() } else { known.join(", ") }))
    })
}

pub(crate) fn surface_param(s: &Session, cmd: &str, p: &Value, key: &str) -> Result<Arc<Tin>> {
    surface(s, str_param(p, key).ok_or_else(|| bad(cmd, format!("`{key}` (a surface name) is required")))?)
}

fn summary(name: &str, t: &Tin) -> Value {
    let z = t.z_range();
    json!({ "name": name, "points": t.points().len(), "triangles": t.triangles().len(),
            "zMin": z.map(|r| r.0), "zMax": z.map(|r| r.1), "area": t.area() })
}

fn max_edge(p: &Value) -> Option<f64> {
    p.get("maxEdge").and_then(Value::as_f64).filter(|v| v.is_finite() && *v > 0.0)
}

fn store(s: &mut Session, name: String, tin: Tin, p: &Value) -> Result<Value> {
    let tin = match max_edge(p) {
        Some(m) => tin.trimmed(m).map_err(EngineError::Other)?,
        None => tin,
    };
    let out = summary(&name, &tin);
    s.surfaces.insert(name, Arc::new(tin));
    Ok(out)
}

fn run_landxmlin(s: &mut Session, p: &Value) -> Result<Value> {
    let path = str_param(p, "path").ok_or_else(|| bad("landxmlin", "`path` (a LandXML file) is required"))?;
    let filters: Vec<String> =
        p.get("names").and_then(Value::as_array).map(|a| a.iter().filter_map(|v| v.as_str()).map(str::to_lowercase).collect()).unwrap_or_default();
    #[cfg(target_arch = "wasm32")]
    {
        let _ = (path, filters);
        Err(bad("landxmlin", "reading files is not available in the browser build"))
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        let bytes =
            crate::capped_read::read_capped(std::path::Path::new(path), MAX_XML_BYTES).map_err(|e| bad("landxmlin", format!("{path}: {e}")))?;
        let text = String::from_utf8_lossy(&bytes);
        let raws = landxml::read(&text, cadcraft_geom::tin::MAX_TIN_POINTS).map_err(|e| bad("landxmlin", e))?;
        let mut loaded = Vec::new();
        for r in raws {
            if !filters.is_empty() && !filters.iter().any(|f| r.name.to_lowercase().contains(f)) {
                continue;
            }
            let tin = Tin::new(r.points, r.faces).map_err(|e| bad("landxmlin", format!("{}: {e}", r.name)))?;
            loaded.push(store(s, r.name, tin, p)?);
        }
        if loaded.is_empty() {
            return Err(bad("landxmlin", "no matching surface with points and faces in the file"));
        }
        Ok(json!({ "surfaces": loaded }))
    }
}

fn xyz(v: &Value) -> Option<Vec3> {
    let a = v.as_array()?;
    let p = Vec3 { x: a.first()?.as_f64()?, y: a.get(1)?.as_f64()?, z: a.get(2)?.as_f64()? };
    p.is_finite().then_some(p)
}

fn run_create(s: &mut Session, p: &Value) -> Result<Value> {
    let name = str_param(p, "name").filter(|n| !n.trim().is_empty()).ok_or_else(|| bad("surface.create", "`name` is required"))?.to_string();
    let arr = p.get("points").and_then(Value::as_array).ok_or_else(|| bad("surface.create", "`points` [[x,y,z],...] is required"))?;
    let pts: Vec<Vec3> =
        arr.iter().map(xyz).collect::<Option<_>>().ok_or_else(|| bad("surface.create", "every point needs three finite numbers [x, y, z]"))?;
    let tin = Tin::from_points(&pts).map_err(|e| bad("surface.create", e))?;
    store(s, name, tin, p)
}

fn run_list(s: &mut Session, _: &Value) -> Result<Value> {
    Ok(json!({ "surfaces": s.surfaces.iter().map(|(n, t)| summary(n, t)).collect::<Vec<_>>() }))
}

fn run_delete(s: &mut Session, p: &Value) -> Result<Value> {
    if bool_or(p, "all", false) {
        let n = s.surfaces.len();
        s.surfaces.clear();
        return Ok(json!({ "deleted": n }));
    }
    let name = str_param(p, "name").ok_or_else(|| bad("surface.delete", "`name` or `all: true` is required"))?;
    s.surfaces.remove(name).map(|_| json!({ "deleted": 1 })).ok_or_else(|| bad("surface.delete", format!("no surface `{name}`")))
}

// ------------------------------------------------------------------------------------------ drawing helpers

/// Makes sure layer `name` exists (with colour index `color`) and returns it, for [`Drawn`] to use.
pub(crate) fn ensure_layer(s: &mut Session, name: &str, color: u8) -> Result<()> {
    let d = s.doc_mut()?;
    if d.layer(name).is_none() {
        let mut l = Layer::new(name);
        l.color = cadcraft_color::Color::Index(color);
        d.layers.push(l);
    }
    Ok(())
}

/// Adds entities on named layers, counting them against [`MAX_DRAWN`]; the current layer is put back at the end.
pub(crate) struct Drawn {
    pub count: usize,
    saved: String,
}

impl Drawn {
    pub fn begin(s: &Session) -> Result<Drawn> {
        Ok(Drawn { count: 0, saved: s.doc()?.header.str("CLAYER", "0").to_string() })
    }

    pub fn end(self, s: &mut Session) -> Result<()> {
        s.doc_mut()?.header.set_str("CLAYER", &self.saved);
        Ok(())
    }

    fn put(&mut self, s: &mut Session, layer: &str, color: u8, kind: EntityKind) -> Result<()> {
        if self.count >= MAX_DRAWN {
            return Err(EngineError::Other(format!("stopped at {MAX_DRAWN} objects; narrow the window or raise the interval")));
        }
        ensure_layer(s, layer, color)?;
        s.doc_mut()?.header.set_str("CLAYER", layer);
        s.add_entity(kind)?;
        self.count += 1;
        Ok(())
    }

    pub fn line(&mut self, s: &mut Session, layer: &str, color: u8, a: Vec2, b: Vec2) -> Result<()> {
        self.put(s, layer, color, line(a, b))
    }

    pub fn polyline(&mut self, s: &mut Session, layer: &str, color: u8, pts: &[Vec2], elevation: f64) -> Result<()> {
        let vertices = pts.iter().map(|p| PolyVertex::new(*p)).collect();
        self.put(s, layer, color, EntityKind::LwPolyline(LwPolyline { vertices, closed: false, const_width: 0.0, elevation, plinegen: false }))
    }

    /// Text centred on `at` (`rotation` in radians).
    pub fn text(&mut self, s: &mut Session, layer: &str, color: u8, at: Vec2, height: f64, rotation: f64, value: &str) -> Result<()> {
        let t = Text {
            insert: v3(at),
            align_pt: Some(v3(at)),
            height,
            value: value.to_string(),
            rotation,
            width_factor: 1.0,
            oblique: 0.0,
            style: "Standard".into(),
            halign: HAlign::Center,
            valign: VAlign::Middle,
        };
        self.put(s, layer, color, EntityKind::Text(t))
    }
}

/// `window` as plan bounds.
fn window(p: &Value, cmd: &str) -> Result<Option<Bounds2>> {
    match p.get("window") {
        None | Some(Value::Null) => Ok(None),
        Some(w) => match w.as_array().map(|a| a.iter().map(point_value).collect::<Vec<_>>()).as_deref() {
            Some([Some(a), Some(b)]) => Ok(Some(Bounds2::new(*a, *b))),
            _ => Err(bad(cmd, "`window` is two corners [[x,y],[x,y]]")),
        },
    }
}

/// The parts of polyline `pts` inside `w` (a segment crossing the border is cut at it).
fn clip_polyline(pts: &[Vec2], w: Bounds2) -> Vec<Vec<Vec2>> {
    let mut out: Vec<Vec<Vec2>> = Vec::new();
    let mut cur: Vec<Vec2> = Vec::new();
    for pair in pts.windows(2) {
        let (Some(&a), Some(&b)) = (pair.first(), pair.get(1)) else { continue };
        // Liang–Barsky.
        let (mut t0, mut t1) = (0.0f64, 1.0f64);
        let d = Vec2::new(b.x - a.x, b.y - a.y);
        let mut inside = true;
        for (p, q) in [(-d.x, a.x - w.min.x), (d.x, w.max.x - a.x), (-d.y, a.y - w.min.y), (d.y, w.max.y - a.y)] {
            if p == 0.0 {
                if q < 0.0 {
                    inside = false;
                }
            } else {
                let r = q / p;
                if p < 0.0 {
                    t0 = t0.max(r);
                } else {
                    t1 = t1.min(r);
                }
            }
        }
        if !inside || t0 > t1 {
            if cur.len() >= 2 {
                out.push(std::mem::take(&mut cur));
            }
            cur.clear();
            continue;
        }
        let (pa, pb) = (a.lerp(b, t0), a.lerp(b, t1));
        if cur.last().is_none_or(|l| !l.near(pa, 1e-9)) {
            if cur.len() >= 2 {
                out.push(std::mem::take(&mut cur));
            }
            cur.clear();
            cur.push(pa);
        }
        cur.push(pb);
    }
    if cur.len() >= 2 {
        out.push(cur);
    }
    out
}

// ------------------------------------------------------------------------------------------ contours, TIN

fn run_contours(s: &mut Session, p: &Value) -> Result<Value> {
    let tin = surface_param(s, "surfacecontours", p, "surface")?;
    let interval = f64_or(p, "interval", 0.0);
    if interval <= 0.0 {
        return Err(bad("surfacecontours", "`interval` (a positive number) is required"));
    }
    let major_every = p.get("majorEvery").and_then(Value::as_u64).unwrap_or(5).max(1);
    let win = window(p, "surfacecontours")?;
    let (zlo, zhi) = tin.z_range().ok_or_else(|| bad("surfacecontours", "the surface has no triangles"))?;
    let (from, to) = (f64_or(p, "from", zlo), f64_or(p, "to", zhi));
    let first = (from / interval).ceil() as i64;
    let last = (to / interval).floor() as i64;
    if last.saturating_sub(first) > 5000 {
        return Err(bad("surfacecontours", "more than 5000 contour levels; raise `interval`"));
    }
    let label_h = f64_or(p, "labelHeight", ((tin.bounds().width().max(tin.bounds().height())) / 400.0).max(1e-6));
    let labels = bool_or(p, "labels", true);
    let minor = str_param(p, "minorLayer").unwrap_or("CONTOUR-MINOR").to_string();
    let major = str_param(p, "majorLayer").unwrap_or("CONTOUR-MAJOR").to_string();
    let mut d = Drawn::begin(s)?;
    let mut verts = 0usize;
    let mut polys = 0usize;
    let result = (|| -> Result<()> {
        for k in first..=last {
            let level = k as f64 * interval;
            let is_major = k.rem_euclid(major_every as i64) == 0;
            let (layer, color) = if is_major { (&major, 2u8) } else { (&minor, 8u8) };
            for chain in tin.contours(level) {
                let parts = match win {
                    Some(w) => clip_polyline(&chain, w),
                    None => vec![chain],
                };
                for part in parts {
                    verts += part.len();
                    polys += 1;
                    d.polyline(s, layer, color, &part, level)?;
                    if labels && is_major && part.len() >= 4 {
                        let (Some(a), Some(b)) = (part.get(part.len() / 2), part.get(part.len() / 2 + 1)) else { continue };
                        let mid = a.mid(*b);
                        let ang = a.angle_to(*b);
                        let ang = if ang.cos() < 0.0 { ang + std::f64::consts::PI } else { ang };
                        d.text(s, &major, 2, mid, label_h, ang, &format!("{level:.0}"))?;
                    }
                }
            }
        }
        Ok(())
    })();
    d.end(s)?;
    result?;
    Ok(json!({ "levels": (last - first + 1).max(0), "polylines": polys, "vertices": verts }))
}

fn run_tin(s: &mut Session, p: &Value) -> Result<Value> {
    let tin = surface_param(s, "surfacetin", p, "surface")?;
    let win = window(p, "surfacetin")?;
    let layer = str_param(p, "layer").unwrap_or("TIN").to_string();
    let mut d = Drawn::begin(s)?;
    let mut seen = std::collections::HashSet::new();
    let result = (|| -> Result<()> {
        for i in 0..tin.triangles().len() {
            let Some(c) = tin.corners(i) else { continue };
            let tb = Bounds2::from_points(c.iter().map(|v| v.xy()));
            if win.is_some_and(|w| !w.intersects(&tb)) {
                continue;
            }
            let Some(t) = tin.triangles().get(i) else { continue };
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                if seen.insert(if a < b { (a, b) } else { (b, a) }) {
                    d.line(s, &layer, 8, c[k].xy(), c[(k + 1) % 3].xy())?;
                }
            }
        }
        Ok(())
    })();
    d.end(s)?;
    result?;
    Ok(json!({ "edges": seen.len() }))
}

// ------------------------------------------------------------------------------------------ volumes

fn region(p: &Value, cmd: &str) -> Result<Option<Vec<Vec2>>> {
    match p.get("region") {
        None | Some(Value::Null) => Ok(None),
        Some(_) => points_param(p, "region").filter(|r| r.len() >= 3).map(Some).ok_or_else(|| bad(cmd, "`region` is at least 3 points [[x,y],...]")),
    }
}

fn run_volume(s: &mut Session, p: &Value) -> Result<Value> {
    let top = surface_param(s, "surfacevolume", p, "top")?;
    let base = surface_param(s, "surfacevolume", p, "base")?;
    let reg = region(p, "surfacevolume")?;
    let v = top.volume_over(&base, reg.as_deref()).map_err(|e| bad("surfacevolume", e))?;
    Ok(json!({ "fill": v.fill, "cut": v.cut, "net": v.net(), "area": v.area, "fillArea": v.fill_area, "cutArea": v.cut_area }))
}

/// A plan path with distance lookup.
pub(crate) struct Path {
    pub pts: Vec<Vec2>,
    cum: Vec<f64>,
}

impl Path {
    pub fn new(pts: Vec<Vec2>) -> Option<Path> {
        let pts = super::distinct_points(&pts);
        if pts.len() < 2 {
            return None;
        }
        let mut cum = vec![0.0];
        for w in pts.windows(2) {
            let (a, b) = (w.first()?, w.get(1)?);
            cum.push(cum.last().copied().unwrap_or(0.0) + a.dist(*b));
        }
        Some(Path { pts, cum })
    }

    pub fn length(&self) -> f64 {
        self.cum.last().copied().unwrap_or(0.0)
    }

    /// Point and unit tangent at distance `d` along the path (clamped to its ends).
    pub fn at(&self, d: f64) -> (Vec2, Vec2) {
        let d = d.clamp(0.0, self.length());
        let i = self.cum.partition_point(|c| *c <= d).saturating_sub(1).min(self.pts.len().saturating_sub(2));
        let (Some(&a), Some(&b)) = (self.pts.get(i), self.pts.get(i + 1)) else { return (Vec2::ZERO, Vec2::new(1.0, 0.0)) };
        let (c0, c1) = (self.cum.get(i).copied().unwrap_or(0.0), self.cum.get(i + 1).copied().unwrap_or(1.0));
        let f = if c1 > c0 { (d - c0) / (c1 - c0) } else { 0.0 };
        (a.lerp(b, f), Vec2::new(b.x - a.x, b.y - a.y).normalized())
    }
}

pub(crate) fn path_param(p: &Value, key: &str, cmd: &str) -> Result<Path> {
    let pts = match p.get(key).and_then(Value::as_array) {
        Some(_) => points_param(p, key),
        None => None,
    };
    let pts = pts.ok_or_else(|| bad(cmd, format!("`{key}` is a path [[x,y],...]")))?;
    // A path given as bulged polyline vertices is not supported here; flatten before calling.
    let flat = Polyline::from_points(&pts, false).tessellate(0.01);
    Path::new(flat).ok_or_else(|| bad(cmd, format!("`{key}` needs at least two different points")))
}

fn run_strips(s: &mut Session, p: &Value) -> Result<Value> {
    let top = surface_param(s, "surfacestrips", p, "top")?;
    let base = surface_param(s, "surfacestrips", p, "base")?;
    let path = path_param(p, "alignment", "surfacestrips")?;
    let start = f64_or(p, "startStation", 0.0);
    let step = f64_or(p, "step", 100.0);
    let width = f64_or(p, "width", 125.0);
    let side = p.get("side").and_then(Value::as_i64).unwrap_or(1).clamp(-1, 1) as f64;
    if step <= 0.0 || width <= 0.0 {
        return Err(bad("surfacestrips", "`step` and `width` must be positive"));
    }
    let (from, to) = (f64_or(p, "from", start), f64_or(p, "to", start + path.length()));
    let mut strips = Vec::new();
    let (mut fill, mut cut) = (0.0, 0.0);
    // Strip edges fall on multiples of `step` in station.
    let mut s0 = from;
    let mut guard = 0;
    while s0 < to - 1e-9 && guard < 200_000 {
        guard += 1;
        let s1 = ((s0 / step).floor() * step + step).min(to);
        let (d0, d1) = (s0 - start, s1 - start);
        let poly = strip_polygon(&path, d0, d1, side, width);
        let v = top.volume_over(&base, Some(&poly)).map_err(|e| bad("surfacestrips", e))?;
        fill += v.fill;
        cut += v.cut;
        strips.push(json!({ "s0": s0, "s1": s1, "fill": v.fill, "cut": v.cut, "net": v.net() }));
        s0 = s1;
    }
    Ok(json!({ "strips": strips, "fill": fill, "cut": cut, "net": fill - cut }))
}

/// The strip between distances `d0` and `d1`: the path on one edge, `width` away on the other (both sides when
/// `side` is 0), ends perpendicular to the path.
pub(crate) fn strip_polygon(path: &Path, d0: f64, d1: f64, side: f64, width: f64) -> Vec<Vec2> {
    let n_steps = (((d1 - d0) / 5.0).ceil() as usize).clamp(1, 2000);
    let ds: Vec<f64> = (0..=n_steps).map(|i| d0 + (d1 - d0) * i as f64 / n_steps as f64).collect();
    let at: Vec<(Vec2, Vec2)> = ds.iter().map(|d| path.at(*d)).collect();
    let off = |k: f64| at.iter().map(|(p, t)| Vec2::new(p.x - t.y * width * k, p.y + t.x * width * k)).collect::<Vec<_>>();
    let mut poly: Vec<Vec2>;
    if side == 0.0 {
        poly = off(1.0);
        poly.extend(off(-1.0).into_iter().rev());
    } else {
        poly = at.iter().map(|(p, _)| *p).collect();
        poly.extend(off(side).into_iter().rev());
    }
    poly
}

#[cfg(test)]
mod tests;
