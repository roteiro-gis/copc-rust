/// Axis-aligned XYZ bounds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds {
    pub min: (f64, f64, f64),
    pub max: (f64, f64, f64),
}

impl Bounds {
    pub fn new(min: (f64, f64, f64), max: (f64, f64, f64)) -> Self {
        Self { min, max }
    }

    pub fn point(x: f64, y: f64, z: f64) -> Self {
        Self {
            min: (x, y, z),
            max: (x, y, z),
        }
    }

    pub fn cube(center: (f64, f64, f64), halfsize: f64) -> Self {
        Self {
            min: (
                center.0 - halfsize,
                center.1 - halfsize,
                center.2 - halfsize,
            ),
            max: (
                center.0 + halfsize,
                center.1 + halfsize,
                center.2 + halfsize,
            ),
        }
    }

    pub fn extend(&mut self, x: f64, y: f64, z: f64) {
        self.min.0 = self.min.0.min(x);
        self.min.1 = self.min.1.min(y);
        self.min.2 = self.min.2.min(z);
        self.max.0 = self.max.0.max(x);
        self.max.1 = self.max.1.max(y);
        self.max.2 = self.max.2.max(z);
    }

    pub fn center(self) -> (f64, f64, f64) {
        (
            (self.min.0 + self.max.0) * 0.5,
            (self.min.1 + self.max.1) * 0.5,
            (self.min.2 + self.max.2) * 0.5,
        )
    }

    pub fn octant(self, octant: u8) -> Self {
        let center = self.center();
        let (min_x, max_x) = if octant & 1 == 0 {
            (self.min.0, center.0)
        } else {
            (center.0, self.max.0)
        };
        let (min_y, max_y) = if octant & 2 == 0 {
            (self.min.1, center.1)
        } else {
            (center.1, self.max.1)
        };
        let (min_z, max_z) = if octant & 4 == 0 {
            (self.min.2, center.2)
        } else {
            (center.2, self.max.2)
        };
        Self {
            min: (min_x, min_y, min_z),
            max: (max_x, max_y, max_z),
        }
    }
}
