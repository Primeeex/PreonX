// PreonX Jubal — Audio System (Rust)
//
// This crate provides Rust-based components for the PreonX audio system.
// It will handle audio format decoding, DSP operations, and other
// tasks where Rust's safety guarantees and performance are advantageous.

/// Returns the version of the jubal crate.
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
