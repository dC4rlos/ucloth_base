#ifndef UCLOTH_DLL_EXPORT_H_
#define UCLOTH_DLL_EXPORT_H_

#ifdef __cplusplus
extern "C"
{
    #endif
    #define ucloth_export __declspec(dllexport)
    #ifdef __cplusplus
}
#endif

#endif // !UCLOTH_DLL_EXPORT_H_