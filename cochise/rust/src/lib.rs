// PreonX Cochise — Asset Pipeline (Rust)
//
// This crate provides Rust-based components for the PreonX asset pipeline.
// It will handle format-specific parsing, encoding/decoding, and other
// tasks where Rust's safety guarantees and ecosystem are advantageous.

/// Returns the version of the cochise crate.
pub fn version() -> &'static str {
    env!("CARGO_PKG_VERSION")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_is_set() {
        assert_eq!(version(), "0.1.0");
    }
}
