//! Common error code

#[derive(Debug)]
pub struct GradDoesNotExist {
    name: String,
    file: String,
    line: u32,
}

impl GradDoesNotExist {
    pub fn new(name: impl Into<String>, file: impl Into<String>, line: u32) -> Self {
        Self {
            name: name.into(),
            file: file.into(),
            line,
        }
    }
}

impl std::fmt::Display for GradDoesNotExist {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Gradient does not exist in variable {} at {}:{}",
            self.name, self.file, self.line
        )
    }
}

/// A macro that attempts to get a gradient of a term, or raise GradDoesNotExist error
macro_rules! try_grad {
    ($term:expr) => {
        $term
            .grad()
            .ok_or_else(|| GradDoesNotExist::new($term.name(), file!(), line!()))?
    };
}
