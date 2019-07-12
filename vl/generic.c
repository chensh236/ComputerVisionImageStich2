#include "generic.h"

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

#if defined(VL_OS_WIN)
#include <Windows.h>
#endif

#if ! defined(VL_DISABLE_THREADS) && defined(VL_THREADS_POSIX)
#include <pthread.h>
#endif

#if defined(VL_OS_MACOSX) || defined(VL_OS_LINUX)
#include <unistd.h>
#endif

#if defined(_OPENMP)
#include <omp.h>
#endif

/* ---------------------------------------------------------------- */
/*                                         Global and thread states */
/* ---------------------------------------------------------------- */

/* Thread state */
typedef struct _VlThreadState
{
  /* errors */
  int lastError ;
  char lastErrorMessage [VL_ERR_MSG_LEN] ;

  /* random number generator */
  VlRand rand ;

  /* time */
#if defined(VL_OS_WIN)
  LARGE_INTEGER ticFreq ;
  LARGE_INTEGER ticMark ;
#else
  clock_t ticMark ;
#endif
} VlThreadState ;

/* Gobal state */
typedef struct _VlState
{
  /* The thread state uses either a mutex (POSIX)
    or a critical section (Win) */
#if defined(VL_DISABLE_THREADS)
  VlThreadState * threadState ;
#else
#if defined(VL_THREADS_POSIX)
  pthread_key_t threadKey ;
  pthread_mutex_t mutex ;
  pthread_t mutexOwner ;
  pthread_cond_t mutexCondition ;
  size_t mutexCount ;
#elif defined(VL_THREADS_WIN)
  DWORD tlsIndex ;
  CRITICAL_SECTION mutex ;
#endif
#endif /* VL_DISABLE_THREADS */

  /* Configurable functions */
  int   (*printf_func)  (char const * format, ...) ;
  void *(*malloc_func)  (size_t) ;
  void *(*realloc_func) (void*,size_t) ;
  void *(*calloc_func)  (size_t, size_t) ;
  void  (*free_func)    (void*) ;

#if defined(VL_ARCH_IX86) || defined(VL_ARCH_X64) || defined(VL_ARCH_IA64)
  VlX86CpuInfo cpuInfo ;
#endif
  vl_size numCPUs ;
  vl_bool simdEnabled ;
  vl_size numThreads ;
} VlState ;

/* Global state instance */
VlState _vl_state ;

/* ----------------------------------------------------------------- */
VL_INLINE VlState * vl_get_state () ;
VL_INLINE VlThreadState * vl_get_thread_specific_state () ;
static void vl_lock_state (void) ;
static void vl_unlock_state (void) ;
static VlThreadState * vl_thread_specific_state_new (void) ;
static void vl_thread_specific_state_delete (VlThreadState * self) ;

/** @brief Get VLFeat version string
 ** @return the library version string.
 **/

char const *
vl_get_version_string ()
{
  return VL_VERSION_STRING ;
}

/** @brief Get VLFeat configuration string.
 ** @return a new configuration string.
 **
 ** The function returns a new string containing a human readable
 ** description of the library configuration.
 **/

char *
vl_configuration_to_string_copy ()
{
  char * string = 0 ;
  int length = 0 ;
  char * staticString = vl_static_configuration_to_string_copy() ;
  char * cpuString =
#if defined(VL_ARCH_IX86) || defined(VL_ARCH_X64) || defined(VL_ARCH_IA64)
  _vl_x86cpu_info_to_string_copy(&vl_get_state()->cpuInfo) ;
#else
  "Generic CPU" ;
#endif
#if defined(DEBUG)
  int const debug = 1 ;
#else
  int const debug = 0 ;
#endif

  while (string == 0) {
    if (length > 0) {
      string = (char*) vl_malloc(sizeof(char) * length) ;
      if (string == NULL) break ;
    }
    length = snprintf(string, length,
                      "VLFeat version %s\n"
                      "    Static config: %s\n"
                      "    %" VL_FMT_SIZE " CPU(s): %s\n"
#if defined(_OPENMP)
                      "    OpenMP: max threads: %d (library: %" VL_FMT_SIZE ")\n"
#endif
                      "    Debug: %s\n",
                      vl_get_version_string (),
                      staticString,
                      vl_get_num_cpus(), cpuString,
#if defined(_OPENMP)
                      omp_get_max_threads(), vl_get_max_threads(),
#endif
                      VL_YESNO(debug)) ;
    length += 1 ;
  }

  if (staticString) vl_free(staticString) ;
  if (cpuString) vl_free(cpuString) ;
  return string ;
}

/** @internal @brief A printf that does not do anything */
static int
do_nothing_printf (char const* format VL_UNUSED, ...)
{
  return 0 ;
}

/** @internal
 ** @brief Lock VLFeat state
 **
 ** The function locks VLFeat global state mutex.
 **
 ** The mutex is recursive: locking multiple times from the same thread
 ** is a valid operations, but requires an equivalent number
 ** of calls to ::vl_unlock_state.
 **
 ** @sa ::vl_unlock_state
 **/

static void
vl_lock_state (void)
{
#if ! defined(VL_DISABLE_THREADS)
#if   defined(VL_THREADS_POSIX)
  VlState * state = vl_get_state () ;
  pthread_t thisThread = pthread_self () ;
  pthread_mutex_lock (&state->mutex) ;
  if (state->mutexCount >= 1 &&
      pthread_equal (state->mutexOwner, thisThread)) {
    state->mutexCount ++ ;
  } else {
    while (state->mutexCount >= 1) {
      pthread_cond_wait (&state->mutexCondition, &state->mutex) ;
    }
    state->mutexOwner = thisThread ;
    state->mutexCount = 1 ;
  }
  pthread_mutex_unlock (&state->mutex) ;
#elif defined(VL_THREADS_WIN)
  EnterCriticalSection (&vl_get_state()->mutex) ;
#endif
#endif
}

/** @internal
 ** @brief Unlock VLFeat state
 **
 ** The function unlocks VLFeat global state mutex.
 **
 ** @sa ::vl_lock_state
 **/

static void
vl_unlock_state (void)
{
#if ! defined(VL_DISABLE_THREADS)
#if   defined(VL_THREADS_POSIX)
  VlState * state = vl_get_state () ;
  pthread_mutex_lock (&state->mutex) ;
  -- state->mutexCount ;
  if (state->mutexCount == 0) {
    pthread_cond_signal (&state->mutexCondition) ;
  }
  pthread_mutex_unlock (&state->mutex) ;
#elif defined(VL_THREADS_WIN)
  LeaveCriticalSection (&vl_get_state()->mutex) ;
#endif
#endif
}

/** @internal
 ** @brief Return VLFeat global state
 **
 ** The function returns a pointer to VLFeat global state.
 **
 ** @return pointer to the global state structure.
 **/

VL_INLINE VlState *
vl_get_state (void)
{
  return &_vl_state ;
}

/** @internal@brief Get VLFeat thread state
 ** @return pointer to the thread state structure.
 **
 ** The function returns a pointer to VLFeat thread state.
 **/

VL_INLINE VlThreadState *
vl_get_thread_specific_state (void)
{
#ifdef VL_DISABLE_THREADS
  return vl_get_state()->threadState ;
#else
  VlState * state ;
  VlThreadState * threadState ;

  vl_lock_state() ;
  state = vl_get_state() ;

#if defined(VL_THREADS_POSIX)
  threadState = (VlThreadState *) pthread_getspecific(state->threadKey) ;
#elif defined(VL_THREADS_WIN)
  threadState = (VlThreadState *) TlsGetValue(state->tlsIndex) ;
#endif

  if (! threadState) {
    threadState = vl_thread_specific_state_new () ;
  }

#if defined(VL_THREADS_POSIX)
  pthread_setspecific(state->threadKey, threadState) ;
#elif defined(VL_THREADS_WIN)
  TlsSetValue(state->tlsIndex, threadState) ;
#endif

  vl_unlock_state() ;
  return threadState ;
#endif
}

/* ---------------------------------------------------------------- */
/** @brief Get the number of CPU cores of the host
 ** @return number of CPU cores.
 **/

vl_size
vl_get_num_cpus (void)
{
  return vl_get_state()->numCPUs ;
}

/** @fn ::vl_set_simd_enabled(vl_bool)
 ** @brief Toggle usage of SIMD instructions
 ** @param x @c true if SIMD instructions are used.
 **
 ** Notice that SIMD instructions are used only if the CPU model
 ** supports them. Note also that data alignment may restrict the use
 ** of such instructions.
 **
 ** @see ::vl_cpu_has_sse2(), ::vl_cpu_has_sse3(), etc.
 **/

void
vl_set_simd_enabled (vl_bool x)
{
  vl_get_state()->simdEnabled = x ;
}

/** @brief Are SIMD instructons enabled?
 ** @return @c true if SIMD instructions are enabled.
 **/

vl_bool
vl_get_simd_enabled (void)
{
  return vl_get_state()->simdEnabled ;
}

/** @brief Check for AVX instruction set
 ** @return @c true if AVX is present.
 **/

vl_bool
vl_cpu_has_avx (void)
{
#if defined(VL_ARCH_IX86) || defined(VL_ARCH_X64) || defined(VL_ARCH_IA64)
  return vl_get_state()->cpuInfo.hasAVX ;
#else
  return VL_FALSE ;
#endif
}

/** @brief Check for SSE3 instruction set
 ** @return @c true if SSE3 is present.
 **/

vl_bool
vl_cpu_has_sse3 (void)
{
#if defined(VL_ARCH_IX86) || defined(VL_ARCH_X64) || defined(VL_ARCH_IA64)
  return vl_get_state()->cpuInfo.hasSSE3 ;
#else
  return VL_FALSE ;
#endif
}

/** @brief Check for SSE2 instruction set
 ** @return @c true if SSE2 is present.
 **/

vl_bool
vl_cpu_has_sse2 (void)
{
#if defined(VL_ARCH_IX86) || defined(VL_ARCH_X64) || defined(VL_ARCH_IA64)
  return vl_get_state()->cpuInfo.hasSSE2 ;
#else
  return VL_FALSE ;
#endif
}

/* ---------------------------------------------------------------- */

/** @brief Get the number of computational threads available to the application
 ** @return number of threads.
 **
 ** This function wraps the OpenMP function @c
 ** omp_get_thread_limit(). If VLFeat was compiled without OpenMP
 ** support, this function returns 1. If VLFeat was compiled with
 ** OpenMP prior to version 3.0 (2008/05), it returns 0.
 **
 ** @sa @ref threads-parallel
 **/

vl_size
vl_get_thread_limit (void)
{
#if defined(_OPENMP)
#if _OPENMP >= 200805
  /* OpenMP version >= 3.0 */
  return omp_get_thread_limit() ;
#else
  return 0 ;
#endif
#else
  return 1 ;
#endif
}

/** @brief Get the maximum number of computational threads used by VLFeat.
 ** @return number of threads.
 **
 ** This function returns the maximum number of thread used by
 ** VLFeat. VLFeat will try to use this number of computational
 ** threads and never exceed it.
 **
 ** This is similar to the OpenMP function @c omp_get_max_threads();
 ** however, it reads a parameter private to VLFeat which is
 ** independent of the value used by the OpenMP library.
 **
 ** If VLFeat was compiled without OpenMP support, this function
 ** returns 1.
 **
 ** @sa vl_set_num_threads(), @ref threads-parallel
 **/

vl_size
vl_get_max_threads (void)
{
#if defined(_OPENMP)
  return vl_get_state()->numThreads ;
#else
  return 1 ;
#endif
}

/** @brief Set the maximum number of threads used by VLFeat.
 ** @param numThreads number of threads to use.
 **
 ** This function sets the maximum number of computational threads
 ** that will be used by VLFeat. VLFeat may in practice use fewer
 ** threads (for example because @a numThreads is larger than the
 ** number of computational cores in the host, or because the number
 ** of threads exceeds the limit available to the application).
 **
 ** If @c numThreads is set to 0, then VLFeat sets the number of
 ** threads to the OpenMP current maximum, obtained by calling @c
 ** omp_get_max_threads().
 **
 ** This function is similar to @c omp_set_num_threads() but changes a
 ** parameter internal to VLFeat rather than affecting OpenMP global
 ** state.
 **
 ** If VLFeat was compiled without, this function does nothing.
 **
 ** @sa vl_get_max_threads(), @ref threads-parallel
 **/

#if defined(_OPENMP)
void
vl_set_num_threads (vl_size numThreads)
{
  if (numThreads == 0) {
    numThreads = omp_get_max_threads() ;
  }
  vl_get_state()->numThreads = numThreads ;
}
#else
void
vl_set_num_threads (vl_size numThreads VL_UNUSED) { }
#endif

/* ---------------------------------------------------------------- */
/** @brief Set last VLFeat error
 ** @param error error code.
 ** @param errorMessage error message format string.
 ** @param ... format string arguments.
 ** @return error code.
 **
 ** The function sets the code and optionally the error message
 ** of the last encountered error. @a errorMessage is the message
 ** format. It uses the @c printf convention and is followed by
 ** the format arguments. The maximum length of the error message is
 ** given by ::VL_ERR_MSG_LEN (longer messages are truncated).
 **
 ** Passing @c NULL as @a errorMessage
 ** sets the error message to the empty string.
 **/

int
vl_set_last_error (int error, char const * errorMessage, ...)
{
  VlThreadState * state = vl_get_thread_specific_state() ;
  va_list args;
  va_start(args, errorMessage) ;
  if (errorMessage) {
#ifdef VL_COMPILER_LCC
    vsprintf(state->lastErrorMessage, errorMessage, args) ;
#else
    vsnprintf(state->lastErrorMessage,
              sizeof(state->lastErrorMessage)/sizeof(char),
              errorMessage, args) ;
#endif
  } else {
    state->lastErrorMessage[0] = 0 ;
  }
  state->lastError = error ;
  va_end(args) ;
  return error ;
}

/** @brief Get the code of the last error
 ** @return error code.
 ** @sa ::vl_get_last_error_message.
 **/

int
vl_get_last_error (void) {
  return vl_get_thread_specific_state()->lastError ;
}

/** @brief Get the last error message
 ** @return pointer to the error message.
 ** @sa ::vl_get_last_error.
 **/

char const *
vl_get_last_error_message (void)
{
  return vl_get_thread_specific_state()->lastErrorMessage ;
}

/* ---------------------------------------------------------------- */
/** @brief Set memory allocation functions
 ** @param malloc_func  pointer to @c malloc.
 ** @param realloc_func pointer to @c realloc.
 ** @param calloc_func  pointer to @c calloc.
 ** @param free_func    pointer to @c free.
 **/

void
vl_set_alloc_func (void *(*malloc_func)  (size_t),
                   void *(*realloc_func) (void*, size_t),
                   void *(*calloc_func)  (size_t, size_t),
                   void  (*free_func)    (void*))
{
  VlState * state ;
  vl_lock_state () ;
  state = vl_get_state() ;
  state->malloc_func  = malloc_func ;
  state->realloc_func = realloc_func ;
  state->calloc_func  = calloc_func ;
  state->free_func    = free_func ;
  vl_unlock_state () ;
}

/** @brief Allocate a memory block
 ** @param n size in bytes of the new block.
 ** @return pointer to the allocated block.
 **
 ** This function allocates a memory block of the specified size.
 ** The synopsis is the same as the POSIX @c malloc function.
 **/

void *
vl_malloc (size_t n)
{
  return (vl_get_state()->malloc_func)(n) ;
  //return (memalign)(32,n) ;
}


/** @brief Reallocate a memory block
 ** @param ptr pointer to a memory block previously allocated.
 ** @param n size in bytes of the new block.
 ** @return pointer to the new block.
 **
 ** This function reallocates a memory block to change its size.
 ** The synopsis is the same as the POSIX @c realloc function.
 **/

void *
vl_realloc (void* ptr, size_t n)
{
  return (vl_get_state()->realloc_func)(ptr, n) ;
}

/** @brief Free and clear a memory block
 ** @param n number of items to allocate.
 ** @param size size in bytes of an item.
 ** @return pointer to the new block.
 **
 ** This function allocates and clears a memory block.
 ** The synopsis is the same as the POSIX @c calloc function.
 **/

void *
vl_calloc (size_t n, size_t size)
{
  return (vl_get_state()->calloc_func)(n, size) ;
}

/** @brief Free a memory block
 ** @param ptr pointer to the memory block.
 **
 ** This function frees a memory block allocated by ::vl_malloc,
 ** ::vl_calloc, or ::vl_realloc. The synopsis is the same as the POSIX
 ** @c malloc function.
 **/

void
vl_free (void *ptr)
{
  (vl_get_state()->free_func)(ptr) ;
}

/* ---------------------------------------------------------------- */

/** @brief Set the printf function
 ** @param printf_func pointer to a @c printf implementation.
 ** Set @c print_func to NULL to disable printf.
 **/

void
vl_set_printf_func (printf_func_t printf_func)
{
  vl_get_state()->printf_func = printf_func ? printf_func : do_nothing_printf ;
}

/** @brief Get the printf function
 ** @return printf_func pointer to the @c printf implementation.
 ** @sa ::vl_set_printf_func.
 **/

printf_func_t
vl_get_printf_func (void) {
  return vl_get_state()->printf_func ;
}

/* ---------------------------------------------------------------- */
/** @brief Get processor time
 ** @return processor time in seconds.
 ** @sa ::vl_tic, ::vl_toc
 **/

double
vl_get_cpu_time ()
{
  #ifdef VL_OS_WIN
  VlThreadState * threadState = vl_get_thread_specific_state() ;
  LARGE_INTEGER mark ;
  QueryPerformanceCounter (&mark) ;
  return (double)mark.QuadPart / (double)threadState->ticFreq.QuadPart ;
#else
  return (double)clock() / (double)CLOCKS_PER_SEC ;
#endif
}

/** @brief Reset processor time reference
 ** The function resets VLFeat TIC/TOC time reference. There is one
 ** such reference per thread.
 ** @sa ::vl_get_cpu_time, ::vl_toc.
 **/

void
vl_tic (void)
{
  VlThreadState * threadState = vl_get_thread_specific_state() ;
#ifdef VL_OS_WIN
  QueryPerformanceCounter (&threadState->ticMark) ;
#else
  threadState->ticMark = clock() ;
#endif
}

/** @brief Get elapsed time since tic
 ** @return elapsed time in seconds.
 **
 ** The function
 ** returns the processor time elapsed since ::vl_tic was called last.
 **
 ** @remark In multi-threaded applications, there is an independent
 ** timer for each execution thread.
 **
 ** @remark On UNIX, this function uses the @c clock() system call.
 ** On Windows, it uses the @c QueryPerformanceCounter() system call,
 ** which is more accurate than @c clock() on this platform.
 **/

double
vl_toc (void)
{
  VlThreadState * threadState = vl_get_thread_specific_state() ;
#ifdef VL_OS_WIN
  LARGE_INTEGER tocMark ;
  QueryPerformanceCounter(&tocMark) ;
  return (double) (tocMark.QuadPart - threadState->ticMark.QuadPart) /
    threadState->ticFreq.QuadPart ;
#else
  return (double) (clock() - threadState->ticMark) / CLOCKS_PER_SEC ;
#endif
}

/* ---------------------------------------------------------------- */
/** @brief Get the default random number generator.
 ** @return random number generator.
 **
 ** The function returns a pointer to the default
 ** random number generator.
 ** There is one such generator per thread.
 **/

VL_EXPORT VlRand *
vl_get_rand (void)
{
  return &vl_get_thread_specific_state()->rand ;
}

/* ---------------------------------------------------------------- */
/*                    Library construction and destruction routines */
/*  --------------------------------------------------------------- */

/** @internal@brief Construct a new thread state object
 ** @return new state structure.
 **/

static VlThreadState *
vl_thread_specific_state_new (void)
{
  VlThreadState * self ;
#if defined(DEBUG)
  printf("VLFeat DEBUG: thread constructor begins.\n") ;
#endif
  self = (VlThreadState*) malloc(sizeof(VlThreadState)) ;
  self->lastError = 0 ;
  self->lastErrorMessage[0] = 0 ;
#if defined(VL_OS_WIN)
  QueryPerformanceFrequency (&self->ticFreq) ;
  self->ticMark.QuadPart = 0 ;
#else
  self->ticMark = 0 ;
#endif
  vl_rand_init (&self->rand) ;

  return self ;
}

/** @internal@brief Delete a thread state structure
 ** @param self thread state object.
 **/

static void
vl_thread_specific_state_delete (VlThreadState * self)
{
#if defined(DEBUG)
  printf("VLFeat DEBUG: thread destructor begins.\n") ;
#endif
  free (self) ;
}
/* ---------------------------------------------------------------- */
/*                                        DLL entry and exit points */
/* ---------------------------------------------------------------- */
/* A constructor and a destructor must be called to initialize or dispose of VLFeat
 * state when the DLL is loaded or unloaded. This is obtained
 * in different ways depending on the operating system.
 */

#if (defined(VL_OS_LINUX) || defined(VL_OS_MACOSX)) && defined(VL_COMPILER_GNUC)
static void vl_constructor () __attribute__ ((constructor)) ;
static void vl_destructor () __attribute__ ((destructor))  ;
#endif

#if defined(VL_OS_WIN)
static void vl_constructor () ;
static void vl_destructor () ;

BOOL WINAPI DllMain(
    HINSTANCE hinstDLL,  // handle to DLL module
    DWORD fdwReason,     // reason for calling function
    LPVOID lpReserved )  // reserved
{
  VlState * state ;
  VlThreadState * threadState ;
  switch (fdwReason) {
    case DLL_PROCESS_ATTACH:
      /* Initialize once for each new process */
      vl_constructor () ;
      break ;

    case DLL_THREAD_ATTACH:
      /* Do thread-specific initialization */
      break ;

    case DLL_THREAD_DETACH:
      /* Do thread-specific cleanup */
#if ! defined(VL_DISABLE_THREADS) && defined(VL_THREADS_WIN)
      state = vl_get_state() ;
      threadState = (VlThreadState*) TlsGetValue(state->tlsIndex) ;
      if (threadState) {
        vl_thread_specific_state_delete (threadState) ;
      }
#endif
      break;

    case DLL_PROCESS_DETACH:
      /* Perform any necessary cleanup */
      vl_destructor () ;
      break;
    }
    return TRUE ; /* Successful DLL_PROCESS_ATTACH */
}
#endif /* VL_OS_WIN */

/* ---------------------------------------------------------------- */
/*                               Library constructor and destructor */
/* ---------------------------------------------------------------- */

/** @internal @brief Initialize VLFeat state */
static void
vl_constructor (void)
{
  VlState * state ;
#if defined(DEBUG)
  printf("VLFeat DEBUG: constructor begins.\n") ;
#endif

  state = vl_get_state() ;

#if ! defined(VL_DISABLE_THREADS)
#if defined(DEBUG)
  printf("VLFeat DEBUG: constructing thread specific state.\n") ;
#endif
#if defined(VL_THREADS_POSIX)
  {
    typedef void (*destructorType)(void * );
    pthread_key_create (&state->threadKey,
                        (destructorType)
                          vl_thread_specific_state_delete) ;
    pthread_mutex_init (&state->mutex, NULL) ;
    pthread_cond_init (&state->mutexCondition, NULL) ;
  }
#elif defined(VL_THREADS_WIN)
  InitializeCriticalSection (&state->mutex) ;
  state->tlsIndex = TlsAlloc () ;
#endif
#else

/* threading support disabled */
#if defined(DEBUG)
  printf("VLFeat DEBUG: constructing the generic thread state instance (threading support disabled).\n") ;
#endif
  vl_get_state()->threadState = vl_thread_specific_state_new() ;
#endif

  state->malloc_func  = malloc ;
  state->realloc_func = realloc ;
  state->calloc_func  = calloc ;
  state->free_func    = free ;
  state->printf_func  = printf ;

  /* on x86 platforms read the CPUID register */
#if defined(VL_ARCH_IX86) || defined(VL_ARCH_X64) || defined(VL_ARCH_IA64)
  _vl_x86cpu_info_init (&state->cpuInfo) ;
#endif

  /* get the number of CPUs */
#if defined(VL_OS_WIN)
  {
    SYSTEM_INFO info;
    GetSystemInfo (&info) ;
    state->numCPUs = info.dwNumberOfProcessors ;
  }
#elif defined(VL_OS_MACOSX) || defined(VL_OS_LINUX)
  state->numCPUs = sysconf(_SC_NPROCESSORS_ONLN) ;
#else
  state->numCPUs = 1 ;
#endif
  state->simdEnabled = VL_TRUE ;

  /* get the number of (OpenMP) threads used by the library */
#if defined(_OPENMP)
  state->numThreads = omp_get_max_threads() ;
#else
  state->numThreads = 1 ;
#endif

#if defined(DEBUG)
  printf("VLFeat DEBUG: constructor ends.\n") ;
#endif
}

/** @internal @brief Destruct VLFeat */
static void
vl_destructor ()
{
  VlState * state ;
#if defined(DEBUG)
  printf("VLFeat DEBUG: destructor begins.\n") ;
#endif

  state = vl_get_state() ;

#if ! defined(VL_DISABLE_THREADS)
#if defined(DEBUG)
  printf("VLFeat DEBUG: destroying a thread specific state instance.\n") ;
#endif
#if   defined(VL_THREADS_POSIX)
  {
    /* Delete the thread state of this thread as the
       destructor is not called by pthread_key_delete or after
       the key is deleted. When the library
       is unloaded, this thread should also be the last one
       using the library, so this is fine.
     */
    VlThreadState * threadState =
       pthread_getspecific(state->threadKey) ;
    if (threadState) {
      vl_thread_specific_state_delete (threadState) ;
      pthread_setspecific(state->threadKey, NULL) ;
    }
  }
  pthread_cond_destroy (&state->mutexCondition) ;
  pthread_mutex_destroy (&state->mutex) ;
  pthread_key_delete (state->threadKey) ;
#elif defined(VL_THREADS_WIN)
 {
    /* Delete the thread state of this thread as the
       destructor is not called by pthread_key_delete or after
       the key is deleted. When the library
       is unloaded, this thread should also be the last one
       using the library, so this is fine.
     */
    VlThreadState * threadState =
      (VlThreadState*) TlsGetValue(state->tlsIndex) ;
    if (threadState) {
      vl_thread_specific_state_delete (threadState) ;
      TlsSetValue(state->tlsIndex, NULL) ;
    }
  }
  TlsFree (state->tlsIndex) ;
  DeleteCriticalSection (&state->mutex) ;
#endif
#else
#if defined(DEBUG)
  printf("VLFeat DEBUG: destroying the generic thread state instance (threading support disabled).\n") ;
#endif
  vl_thread_specific_state_delete(vl_get_state()->threadState) ;
#endif

#if defined(DEBUG)
  printf("VLFeat DEBUG: destructor ends.\n") ;
#endif
}
