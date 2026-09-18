/*
 * strlookup.c — String table stub (reimplements strlkup.dll)
 *
 * The GOG release ships without lang.txt and all UI text is hardcoded
 * English in the binary. StrLookupCreate returns NULL; every Find/Format
 * returns NULL; callers fall through to their hardcoded strings.
 *
 * If lang.txt format is ever confirmed, a real parser can go here.
 */

#include "strlookup.h"
#include <stddef.h>

StrLookup  *StrLookupCreate(const char *filename) { (void)filename; return NULL; }
const char *StrLookupFind(const StrLookup *sl, const char *key) { (void)sl; (void)key; return NULL; }
const char *StrLookupFormat(const StrLookup *sl, const char *key, ...) { (void)sl; (void)key; return NULL; }
void        StrLookupDestroy(StrLookup *sl) { (void)sl; }
