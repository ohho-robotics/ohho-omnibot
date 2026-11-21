# Fixture source

`omnibot.descriptor.golden.json` is a byte-identical vendored copy of the
canonical ohho-sdk golden descriptor for OmniBot:

- Origin repo: `ohho-os/ohho-sdk` (read-only reference)
- Origin path: `packages/schemas/ohho.descriptor.v1/golden/omnibot.json`

It is the expected-value fixture for the drift test in
`../test_descriptor.py`. Update it only from a new ohho-sdk golden export,
never by hand.
