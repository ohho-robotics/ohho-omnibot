# Gitleaks: pending Supabase anon-key rotation (OHH-190)

Prepared 2026-10-14. The Supabase anon key has **not** been rotated or revoked. gitleaks stays red on this historical finding until it is.

Do not rewrite git history. The value is omitted here on purpose.

## Credential

| | |
|---|---|
| Rule | `jwt` |
| File | `omnibot-vr/Assets/Resources/vr_manifest.json` |
| Line | 21 (`auth.anonKey`) |
| Commit | `187dec2cf0daa2145ef771bf696a0e5bed6dd865` (2026-09-25, "Initial split repository") |
| What | Supabase anon key (JWT) for `https://ucnyuappgtcpolqdaovv.supabase.co` |
| Current tree | `auth.anonKey` is an empty string. The headset reads a replacement from the gitignored `omnibot-vr/Assets/Resources/vr_manifest_local.json` (see [omnibot-vr/README.md](../../omnibot-vr/README.md), section 4). |
| Rotation | Not done |

Fingerprint (string only):

```
187dec2cf0daa2145ef771bf696a0e5bed6dd865:omnibot-vr/Assets/Resources/vr_manifest.json:jwt:21
```

A scan of this repo with gitleaks 8.30.1, the same command as [`.github/workflows/gitleaks.yml`](../../.github/workflows/gitleaks.yml) (`gitleaks git . --config .gitleaks.toml --redact --no-banner`), reported this fingerprint and one unrelated false positive. The false positive is already in `.gitleaksignore`. This fingerprint is not.

## After the key is rotated

1. In the Supabase dashboard for `ucnyuappgtcpolqdaovv`, revoke the anon key that was committed in that file and issue a new one. Record the date and the credential name on Linear OHH-190. Do not record either key value.
2. Put the new key only in gitignored `omnibot-vr/Assets/Resources/vr_manifest_local.json`, and in any deployed client outside this repo that still has the old key. Do not commit it.
3. Confirm a fresh gitleaks report has no new jwt finding. The historical finding above remains until it is ignored.
4. Append this exact line to `.gitleaksignore` (leave the existing DevAgent fingerprint in place; do not add a path-wide or rule-wide allowlist):

```
187dec2cf0daa2145ef771bf696a0e5bed6dd865:omnibot-vr/Assets/Resources/vr_manifest.json:jwt:21
```

5. Re-run `gitleaks git . --config .gitleaks.toml --redact --no-banner`. It should exit 0.
