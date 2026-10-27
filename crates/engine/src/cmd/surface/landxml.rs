//! A reader for the surface part of LandXML: `<Surface name="…"><Definition><Pnts><P id="1">N E Z</P>…</Pnts>
//! <Faces><F>1 2 3</F>…</Faces>`. It scans the text for those tags only; everything else in the file is ignored.

use cadcraft_geom::Vec3;

/// A surface as the file holds it.
pub struct RawSurface {
    pub name: String,
    pub points: Vec<Vec3>,
    pub faces: Vec<[u32; 3]>,
}

/// Most surfaces read from one file.
const MAX_SURFACES: usize = 500;

/// `<tag attrs>inner</tag>` blocks of `xml`, in order (self-closing tags have an empty inner part).
fn blocks<'a>(xml: &'a str, tag: &str) -> Vec<(&'a str, &'a str)> {
    let open = format!("<{tag}");
    let close = format!("</{tag}>");
    let mut out = Vec::new();
    let mut at = 0;
    while let Some(i) = xml.get(at..).and_then(|r| r.find(&open)).map(|i| i + at) {
        let after = i + open.len();
        let next = xml.as_bytes().get(after).copied();
        if !matches!(next, Some(b' ' | b'>' | b'/' | b'\t' | b'\r' | b'\n')) {
            at = after;
            continue;
        }
        let Some(gt) = xml.get(after..).and_then(|r| r.find('>')).map(|g| g + after) else { break };
        let attrs = xml.get(after..gt).unwrap_or("");
        if attrs.ends_with('/') {
            out.push((attrs, ""));
            at = gt + 1;
            continue;
        }
        let end = xml.get(gt + 1..).and_then(|r| r.find(&close)).map(|e| e + gt + 1);
        let inner = xml.get(gt + 1..end.unwrap_or(xml.len())).unwrap_or("");
        out.push((attrs, inner));
        at = end.map_or(xml.len(), |e| e + close.len());
    }
    out
}

fn attr(attrs: &str, name: &str) -> Option<String> {
    for q in ['"', '\''] {
        let key = format!("{name}={q}");
        let mut from = 0;
        while let Some(i) = attrs.get(from..).and_then(|r| r.find(&key)).map(|i| i + from) {
            let starts_word = i == 0 || attrs.as_bytes().get(i - 1).is_some_and(|b| b.is_ascii_whitespace());
            let v = i + key.len();
            if starts_word && let Some(len) = attrs.get(v..).and_then(|r| r.find(q)) {
                return Some(unescape(attrs.get(v..v + len).unwrap_or("")));
            }
            from = v;
        }
    }
    None
}

fn unescape(s: &str) -> String {
    s.replace("&lt;", "<").replace("&gt;", ">").replace("&quot;", "\"").replace("&apos;", "'").replace("&amp;", "&")
}

/// Every surface in a LandXML document. A point's `id` may be any string; faces refer to them by it.
pub fn read(xml: &str, max_points: usize) -> Result<Vec<RawSurface>, String> {
    let mut out = Vec::new();
    for (attrs, inner) in blocks(xml, "Surface") {
        if out.len() >= MAX_SURFACES {
            return Err(format!("more than {MAX_SURFACES} surfaces in the file"));
        }
        let name = attr(attrs, "name").unwrap_or_else(|| format!("Surface{}", out.len() + 1));
        let mut ids: std::collections::HashMap<&str, u32> = std::collections::HashMap::new();
        let mut points: Vec<Vec3> = Vec::new();
        for (pa, body) in blocks(inner, "P") {
            let nums: Vec<f64> = body.split_whitespace().filter_map(|t| t.parse::<f64>().ok()).collect();
            let (Some(&n), Some(&e), Some(&z)) = (nums.first(), nums.get(1), nums.get(2)) else { continue };
            if !(n.is_finite() && e.is_finite() && z.is_finite()) {
                continue;
            }
            if points.len() >= max_points {
                return Err(format!("surface `{name}` has more than {max_points} points"));
            }
            // The id is borrowed from the attribute text of this block, which lives in `inner`.
            let id = id_of(pa);
            let idx = points.len() as u32;
            if let Some(id) = id {
                ids.insert(id, idx);
            }
            points.push(Vec3 { x: e, y: n, z });
        }
        let mut faces: Vec<[u32; 3]> = Vec::new();
        for (fa, body) in blocks(inner, "F") {
            if attr(fa, "i").as_deref() == Some("1") {
                continue; // an invisible face
            }
            let refs: Vec<u32> = body.split_whitespace().filter_map(|t| ids.get(t).copied()).collect();
            if let [a, b, c] = refs[..] {
                faces.push([a, b, c]);
            }
        }
        if !points.is_empty() && !faces.is_empty() {
            out.push(RawSurface { name, points, faces });
        }
    }
    Ok(out)
}

/// The `id="…"` value as a slice of the attribute text.
fn id_of(attrs: &str) -> Option<&str> {
    for q in ['"', '\''] {
        let key = format!("id={q}");
        let mut from = 0;
        while let Some(i) = attrs.get(from..).and_then(|r| r.find(&key)).map(|i| i + from) {
            let v = i + key.len();
            if (i == 0 || attrs.as_bytes().get(i - 1).is_some_and(|b| b.is_ascii_whitespace()))
                && let Some(len) = attrs.get(v..).and_then(|r| r.find(q))
            {
                return attrs.get(v..v + len);
            }
            from = v;
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    const XML: &str = r#"<?xml version="1.0"?><LandXML><Surfaces>
      <Surface name="Top &amp; Base" desc="x"><Definition surfType="TIN"><Pnts>
        <P id="1">10 20 5.5</P><P id="2">10 30 6</P><P id="7">20 20 7</P><P id="9">20 30 8</P></Pnts>
        <Faces><F>1 2 7</F><F i="1">2 7 9</F><F>2 9 7</F><F>1 2 99</F></Faces></Definition></Surface>
      <Surface name="Empty"><Definition><Pnts/></Definition></Surface></Surfaces></LandXML>"#;

    #[test]
    fn reads_points_and_visible_faces() {
        let s = read(XML, 100).unwrap();
        assert_eq!(s.len(), 1);
        assert_eq!(s[0].name, "Top & Base");
        assert_eq!(s[0].points.len(), 4);
        // Northing first, easting second: x = easting.
        assert_eq!((s[0].points[0].x, s[0].points[0].y, s[0].points[0].z), (20.0, 10.0, 5.5));
        assert_eq!(s[0].faces, vec![[0, 1, 2], [1, 3, 2]]);
    }

    #[test]
    fn caps_and_garbage_do_not_panic() {
        assert!(read(XML, 2).is_err());
        assert!(read("<Surface name=\"a\"><Pnts><P id=\"1\">1 2", 10).unwrap().is_empty());
        assert!(read("<<<>>><Surface", 10).unwrap().is_empty());
        assert!(read("", 10).unwrap().is_empty());
    }
}
