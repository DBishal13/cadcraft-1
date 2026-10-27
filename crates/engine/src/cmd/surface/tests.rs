use serde_json::json;

use crate::Session;

/// A 21 × 21 grid of 5-unit cells on the plane z = 100 + x / 10, named `ground`.
fn ground(s: &mut Session) {
    let mut pts = Vec::new();
    for i in 0..=20 {
        for j in 0..=20 {
            let (x, y) = (f64::from(i) * 5.0, f64::from(j) * 5.0);
            pts.push(json!([x, y, 100.0 + x / 10.0]));
        }
    }
    s.execute("surface.create", &json!({ "name": "ground", "points": pts })).unwrap();
}

/// `design` is `ground` raised by 2 where y is above 50 and lowered by 1 below it.
fn design(s: &mut Session) {
    let mut pts = Vec::new();
    for i in 0..=20 {
        for j in 0..=20 {
            let (x, y) = (f64::from(i) * 5.0, f64::from(j) * 5.0);
            pts.push(json!([x, y, 100.0 + x / 10.0 + if y > 50.0 { 2.0 } else { -1.0 }]));
        }
    }
    s.execute("surface.create", &json!({ "name": "design", "points": pts })).unwrap();
}

#[test]
fn surfaces_are_listed_and_deleted() {
    let mut s = Session::new();
    ground(&mut s);
    let l = s.execute("surface.list", &json!({})).unwrap();
    assert_eq!(l["surfaces"][0]["name"], "ground");
    assert_eq!(l["surfaces"][0]["points"], 441);
    assert!((l["surfaces"][0]["area"].as_f64().unwrap() - 10_000.0).abs() < 1e-6);
    s.execute("surface.delete", &json!({ "name": "ground" })).unwrap();
    assert!(s.execute("surface.delete", &json!({ "name": "ground" })).is_err());
}

#[test]
fn contours_are_drawn_on_major_and_minor_layers() {
    let mut s = Session::new();
    ground(&mut s);
    let r = s.execute("surfacecontours", &json!({ "surface": "ground", "interval": 0.5, "majorEvery": 2 })).unwrap();
    assert!(r["polylines"].as_u64().unwrap() >= 2, "{r}");
    let d = s.doc().unwrap();
    assert!(d.layer("CONTOUR-MAJOR").is_some() && d.layer("CONTOUR-MINOR").is_some());
    // The current layer is back to what it was.
    assert_eq!(d.header.str("CLAYER", "0"), "0");
    let before = d.model.len();
    s.execute("surfacecontours", &json!({ "surface": "ground", "interval": 0.5, "window": [[0, 0], [20, 20]], "labels": false })).unwrap();
    assert!(s.doc().unwrap().model.len() > before);
    assert!(s.execute("surfacecontours", &json!({ "surface": "ground", "interval": 0 })).is_err());
    assert!(s.execute("surfacecontours", &json!({ "surface": "nope", "interval": 1 })).is_err());
}

#[test]
fn volume_between_surfaces_and_in_a_region() {
    let mut s = Session::new();
    ground(&mut s);
    design(&mut s);
    let v = s.execute("surfacevolume", &json!({ "top": "design", "base": "ground" })).unwrap();
    // The surfaces differ by +2 above y = 55 and -1 below y = 50, ramping between (a 5-unit band, crossing zero a
    // third of the way up): fill 2 * 45 * 100 + 1000 / 3, cut 50 * 100 + 250 / 3.
    assert!((v["fill"].as_f64().unwrap() - (9_000.0 + 1000.0 / 3.0)).abs() < 1e-6, "{v}");
    assert!((v["cut"].as_f64().unwrap() - (5_000.0 + 250.0 / 3.0)).abs() < 1e-6, "{v}");
    let r = s.execute("surfacevolume", &json!({ "top": "design", "base": "ground", "region": [[0, 60], [10, 60], [10, 80], [0, 80]] })).unwrap();
    assert!((r["fill"].as_f64().unwrap() - 400.0).abs() < 1e-6 && r["cut"].as_f64().unwrap().abs() < 1e-9, "{r}");
}

#[test]
fn strips_add_up_to_the_whole_volume() {
    let mut s = Session::new();
    ground(&mut s);
    design(&mut s);
    // An alignment along y = 25 from x = 0 to 100; strips 25 units wide to the left (north) of it reach y = 50.
    let r = s
        .execute("surfacestrips", &json!({ "top": "design", "base": "ground", "alignment": [[0, 25], [100, 25]], "step": 20, "width": 20 }))
        .unwrap();
    assert_eq!(r["strips"].as_array().unwrap().len(), 5);
    assert!((r["cut"].as_f64().unwrap() - 20.0 * 100.0).abs() < 1e-6, "{r}");
    assert!(r["fill"].as_f64().unwrap().abs() < 1e-9);
    // Strip edges fall on multiples of the step in station.
    let r = s
        .execute(
            "surfacestrips",
            &json!({ "top": "design", "base": "ground", "alignment": [[0, 25], [100, 25]], "startStation": 10, "step": 50, "width": 10, "side": -1 }),
        )
        .unwrap();
    assert_eq!(r["strips"][0]["s0"], 10.0);
    assert_eq!(r["strips"][0]["s1"], 50.0);
}

#[test]
fn cross_sections_report_end_areas() {
    let mut s = Session::new();
    ground(&mut s);
    design(&mut s);
    // An alignment along x = 50 running north from y = 0 (station 20 at y = 0); sections 30 each side.
    let before = s.doc().unwrap().model.len();
    let r = s
        .execute(
            "surfacesections",
            &json!({ "alignment": [[50, 0], [50, 100]], "surfaces": ["design", "ground"], "interval": 40, "startStation": 20, "width": 30, "at": [200, 0], "columns": 2 }),
        )
        .unwrap();
    assert!(s.doc().unwrap().model.len() > before + 20);
    // Stations 40, 80 and 120 lie on the 20..120 alignment.
    assert_eq!(r["count"], 3);
    // Station 40 is at y = 20, where the design is 1 below the ground over the 60-wide section.
    assert!((r["sections"][0]["cutArea"].as_f64().unwrap() - 60.0).abs() < 1e-6, "{r}");
    assert!(r["sections"][0]["fillArea"].as_f64().unwrap().abs() < 1e-9);
    // Station 120 is at y = 100, where it is 2 above.
    assert!((r["sections"][2]["fillArea"].as_f64().unwrap() - 120.0).abs() < 1e-6, "{r}");
    assert!(r["volume"]["fill"].as_f64().unwrap() > 0.0);
}

#[test]
fn profile_runs_along_a_path() {
    let mut s = Session::new();
    ground(&mut s);
    let r = s.execute("surfacesection", &json!({ "path": [[-10, 50], [110, 50]], "surfaces": ["ground"], "at": [0, -300] })).unwrap();
    assert_eq!(r["surfaces"][0]["runs"], 1);
    assert!(r["objects"].as_u64().unwrap() > 10);
    assert!(s.execute("surfacesection", &json!({ "path": [[500, 500], [600, 500]], "surfaces": ["ground"], "at": [0, 0] })).is_err());
}

#[test]
fn landxml_round_trip_and_hostile_files() {
    let dir = std::env::temp_dir().join(format!("cadcraft-landxml-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let f = dir.join("t.xml");
    std::fs::write(
        &f,
        r#"<LandXML><Surfaces><Surface name="Test Surface"><Definition><Pnts><P id="1">0 0 1</P><P id="2">0 10 2</P><P id="3">10 0 3</P><P id="4">10 10 4</P></Pnts><Faces><F>1 2 3</F><F>2 4 3</F></Faces></Definition></Surface></Surfaces></LandXML>"#,
    )
    .unwrap();
    let mut s = Session::new();
    let r = s.execute("landxmlin", &json!({ "path": f.to_str().unwrap(), "names": ["test surf"] })).unwrap();
    assert_eq!(r["surfaces"][0]["triangles"], 2);
    assert!((s.surfaces["Test Surface"].area() - 100.0).abs() < 1e-9);
    assert!(s.execute("landxmlin", &json!({ "path": f.to_str().unwrap(), "names": ["nothing"] })).is_err());
    assert!(s.execute("landxmlin", &json!({ "path": dir.join("missing.xml").to_str().unwrap() })).is_err());
    std::fs::write(&f, "<Surface name=\"a\"><Pnts><P id=\"1\">NaN inf 1</P></Pnts><Faces><F>1 1 1</F></Faces>").unwrap();
    assert!(s.execute("landxmlin", &json!({ "path": f.to_str().unwrap() })).is_err());
    std::fs::remove_dir_all(&dir).ok();
}

#[test]
fn hostile_params_never_panic() {
    let mut s = Session::new();
    ground(&mut s);
    for cmd in [
        "surfacecontours",
        "surfacetin",
        "surfacesection",
        "surfacesections",
        "surfacevolume",
        "surfacestrips",
        "surface.create",
        "landxmlin",
        "surface.delete",
    ] {
        for p in [
            json!({}),
            json!({ "surface": "ground", "interval": 1e-300, "top": "ground", "base": "ground" }),
            json!({ "surface": "ground", "interval": 1e300, "window": [[1e308, 1e308], [-1e308, -1e308]] }),
            json!({ "alignment": [[0, 0], [0, 0]], "path": [[0, 0]], "surfaces": [], "points": [[f64::MAX, 0, 0]], "name": "" }),
            json!({ "top": "ground", "base": "ground", "alignment": [[0, 0], [1e9, 1e9]], "step": 1e-9, "width": 1e300 }),
            json!({ "points": [[0, 0, 0], [1, 1, 1]], "name": "x", "region": [[0, 0]] }),
        ] {
            let _ = s.execute(cmd, &p);
        }
    }
}
