

struct Rectangle{
    base: u32,
    height: u32
}

struct Rectangle{
    base1: u32,
    base2: u32,
    height: u32,
}


trait constructor {
    fn new(&self) -> Box<dyn Self>;
}

impl constructor for Rectangle{
    fn new(b:u32, h:u32){
        return Rectangle{b, h}
    }
}