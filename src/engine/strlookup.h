#ifndef STRLOOKUP_H
#define STRLOOKUP_H

/*
 * strlookup.h — String table system (reimplements strlkup.dll)
 *
 * The original game loads lang.txt at startup via StrLookupCreate.
 * The GOG release ships without lang.txt — all UI text is hardcoded
 * English in the binary. When the file is absent, every StrLookupFind
 * returns NULL and the game uses its fallback strings throughout.
 *
 * File format (Windows INI style, CRLF or LF):
 *   [Language]        ← section header, ignored
 *   KEY=Value text    ← key/value pair
 *   ; comment         ← semicolon or hash comment, ignored
 *   # comment
 *
 * Keys are case-insensitive. Values may contain any characters including =.
 */

typedef struct StrLookup StrLookup;

/* Open lang.txt (or any KEY=VALUE file) via the VFS.
 * Returns NULL if the file is not found — callers must handle NULL. */
StrLookup  *StrLookupCreate(const char *filename);

/* Look up a key. Returns the value string, or NULL if not found.
 * The returned pointer is valid until StrLookupDestroy is called. */
const char *StrLookupFind(const StrLookup *sl, const char *key);

/* Like StrLookupFind, but formats the value with printf-style args.
 * Returns a pointer to an internal static buffer — not thread-safe.
 * Returns NULL if the key is not found. */
const char *StrLookupFormat(const StrLookup *sl, const char *key, ...);

void StrLookupDestroy(StrLookup *sl);

#endif /* STRLOOKUP_H */
