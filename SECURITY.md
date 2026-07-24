# Security Policy

## Reporting a Vulnerability

If you discover a security vulnerability within PreonX, please report it responsibly.

**Do not open a public GitHub issue for security vulnerabilities.**

### How to Report

Send an email to **primeeex23@gmail.com** with:

1. A description of the vulnerability
2. Steps to reproduce the issue
3. Potential impact assessment
4. Any suggested fixes (if applicable)

### What to Expect

- **Acknowledgment**: You will receive an acknowledgment within 48 hours.
- **Assessment**: The team will assess the vulnerability and determine its severity.
- **Resolution**: We will work on a fix and coordinate disclosure with you.
- **Credit**: Unless you prefer to remain anonymous, we will credit you in the release notes.

## Scope

This security policy applies to:

- The PreonX engine code in this repository
- Build system configurations that could affect the security of built artifacts
- Documentation that includes code examples with security implications

### Out of Scope

- Third-party dependencies (report these to their respective maintainers)
- Issues that require physical access to the target system
- Social engineering attacks

## Supported Versions

| Version | Supported |
|---------|-----------|
| 0.1.x   | Yes       |

## Security Best Practices

When using PreonX in your project:

- Always build with the latest version of your compiler.
- Enable address sanitizers (`-fsanitize=address`) during development.
- Enable undefined behavior sanitizer (`-fsanitize=undefined`) during development.
- Use the Release build configuration for production.
- Review third-party dependencies regularly.

## Disclosure Policy

We follow coordinated disclosure:

1. Reporter reports the vulnerability privately.
2. We have 90 days to develop a fix.
3. We release the fix and publish a security advisory.
4. Reporter is credited (unless they prefer otherwise).
