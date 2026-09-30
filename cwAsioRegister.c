/** @file       cwAsioRegister.c
 *  @brief      Register or unregister a cwASIO driver by calling its exported functions
 *  @author     Axel Holzinger, Stefan Heinzmann
 *  @version    1.0
 *  @date       2026
 *  @copyright  See file LICENSE in toplevel directory
 *
 * Usage: cwAsioRegister [-u] DRIVER NAME
 *
 * Loads the driver shared object DRIVER and calls its registerDriver(NAME), or
 * unregisterDriver(NAME) when -u is given. Needs write access to /etc/cwASIO.
 */

#include <dlfcn.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef int (*RegFunc)(char const *name);

static char const *explain(int err, int unregister) {
    switch(err) {
        case EEXIST:    return "the name is already registered for a different driver";
        case EACCES:    return unregister ? "the name is registered for a different driver" : strerror(err);
        case ENODEV:    return "no driver is registered under this name";
        case ENOTEMPTY: return "the registration directory contains subdirectories";
        default:        return strerror(err);
    }
}

int main(int argc, char *argv[]) {
    int unregister = argc > 1 && 0 == strcmp(argv[1], "-u");
    if(argc != 3 + unregister) {
        fprintf(stderr, "Usage: %s [-u] DRIVER NAME\n", argv[0]);
        return 2;
    }
    char const *driver = argv[1 + unregister];
    char const *name = argv[2 + unregister];

    // registerDriver() records the path the driver was loaded from, so it must be absolute
    char path[PATH_MAX];
    if(!realpath(driver, path)) {
        fprintf(stderr, "%s: %s: %s\n", argv[0], driver, strerror(errno));
        return 1;
    }
    void *lib = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    if(!lib) {
        fprintf(stderr, "%s: %s\n", argv[0], dlerror());
        return 1;
    }
    char const *funcName = unregister ? "unregisterDriver" : "registerDriver";
    RegFunc func = (RegFunc)dlsym(lib, funcName);
    if(!func) {
        fprintf(stderr, "%s: %s is not a cwASIO driver: %s\n", argv[0], path, dlerror());
        return 1;
    }
    int err = func(name);
    if(err) {
        fprintf(stderr, "%s: %s '%s' failed: %s\n", argv[0], funcName, name, explain(err, unregister));
        return 1;
    }
    return 0;
}
