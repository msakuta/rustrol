pub(crate) struct Xor128 {
    x: u32,
    y: u32,
    z: u32,
    w: u32,
}

impl Xor128 {
    pub fn new(seed: u32) -> Self {
        let mut ret = Xor128 {
            x: 294742812,
            y: 3863451937,
            z: 2255883528,
            w: 824091511,
        };
        if 0 < seed {
            ret.x ^= seed;
            ret.y ^= ret.x;
            ret.z ^= ret.y;
            ret.w ^= ret.z;
            ret.nexti();
        }
        ret.nexti();
        ret
    }

    pub fn nexti(&mut self) -> u32 {
        // T = (I + L^a)(I + R^b)(I + L^c)
        // a = 13, b = 17, c = 5
        let t = self.x ^ (self.x << 15);
        self.x = self.y;
        self.y = self.z;
        self.z = self.w;
        self.w ^= (self.w >> 21) ^ (t ^ (t >> 4));
        self.w
    }

    pub fn next(&mut self) -> f64 {
        self.nexti() as f64 / 0xffffffffu32 as f64
    }
}
