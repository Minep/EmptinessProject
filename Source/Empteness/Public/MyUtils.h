#ifndef CB9B9DAD_931D_4CB1_8E7C_A5A4DEA2D171
#define CB9B9DAD_931D_4CB1_8E7C_A5A4DEA2D171

#define ACTOR_TAG(tag) TEXT("Actor." tag)

DECLARE_LOG_CATEGORY_EXTERN(LogEmptiness, Log, All);

#define EM_LOG(Level, Fmt, ...)\
    UE_LOG(LogEmptiness, Level, TEXT(Fmt), ## __VA_ARGS__)

#endif /* CB9B9DAD_931D_4CB1_8E7C_A5A4DEA2D171 */
