#ifndef UCLOTH_DLL_HANDLES_H_
#define UCLOTH_DLL_HANDLES_H_

#define DECLARE_HANDLE(name) \
    typedef struct name##__  \
    {                        \
        int unused;          \
    } * name

#define DECLARE_CONST_HANDLE(name) \
    typedef struct name##__        \
    {                              \
        int unused;                \
    } const* name

DECLARE_HANDLE(PBDSystemHandle);
DECLARE_HANDLE(WorldHandle);
DECLARE_CONST_HANDLE(ClothHandle);

#endif // !UCLOTH_DLL_HANDLES_H_