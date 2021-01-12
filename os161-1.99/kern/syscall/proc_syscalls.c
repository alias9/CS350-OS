#include <types.h>
#include <kern/errno.h>
#include <kern/unistd.h>
#include <kern/wait.h>
#include <lib.h>
#include <syscall.h>
#include <current.h>
#include <proc.h>
#include <thread.h>
#include <addrspace.h>
#include <copyinout.h>
#include "opt-A2.h"
#include <vfs.h>
#include <kern/fcntl.h>

#if OPT_A2
/* sys_execv handler */
int sys_execv(userptr_t program, userptr_t args) {
  struct addrspace *as;
  struct vnode *v;
  vaddr_t entrypoint, stackptr;
  int result;

  (void)args;
  if(program == NULL) return ENOENT;

  /* 1. copy program name (path) to the kernel */
  char kprog[128];
  if(kprog == NULL) return ENOMEM;  // just in case

  size_t lprog;
  result = copyinstr((const_userptr_t)program, (void *)kprog, 128 * sizeof(char), &lprog);
  if(result) return result;
  // kprintf("$$$$$ kprog: %s\n", kprog);

  /* 2. count # of args */
  int argc = 0;
  char **argv = (char **)args;   // userptr_t --> char **
  if(argv == NULL)  return EFAULT;

  while(argv[argc]) {++argc;}
  if(argc > 128) return E2BIG;  // in real world, ARG_MAX
  // kprintf("@@@@@@@@ argc: %d\n", argc);

  /* 3. copy every argument to the kernel stack */
  char *kargs[128];
  size_t l;
  for (int i = 0; i < argc; ++i) {
    int arglen = strlen(argv[i]) + 1;
    kargs[i] = kmalloc(arglen * sizeof(char));
    result = copyinstr((const_userptr_t)argv[i], (void *)kargs[i], arglen * sizeof(char), &l);
    if(result) return result;
  }
  kargs[argc] = NULL;

  /* Open the file. */
  result = vfs_open(kprog, O_RDONLY, 0, &v);
  if (result) {
      return result;
  }

  /* Create a new address space. */
  as = as_create();
  if (as ==NULL) {
      vfs_close(v);
      return ENOMEM;
  }

  /* Switch to it and activate it. */
  struct addrspace *oldas = curproc_setas(as);
  as_activate();    // Invalidates TLB entries

  /* Load the executable. */
  result = load_elf(v, &entrypoint);
  if (result) {
      /* p_addrspace will go away when curproc is destroyed */
      vfs_close(v);
      return result;
  }

  /* Done with the file now. */
  vfs_close(v);

  /* Define the user stack in the address space */
  result = as_define_stack(as, &stackptr);
  if (result) {
      /* p_addrspace will go away when curproc is destroyed */
      return result;
  }

  // stackptr should be 0x8000 0000 [USERSTACK] now
  /* 4. copy every arg to the user stack */
  char *argptr[128];
  for (int i = 0; i < argc; ++i) {
    int arglen = strlen(kargs[i]) + 1;
    stackptr -= ROUNDUP(arglen * sizeof(char), 8);
    result = copyoutstr((const char *)kargs[i], (userptr_t)stackptr, arglen * sizeof(char), &l);
    if(result)  return result;
    argptr[i] = (char *)stackptr;
  }

  /* 5. copy [array of ptrs to the args] to the user stack */
  stackptr -= ROUNDUP(sizeof(char *) * (argc + 1), 8);  // set space for argptr[]
  vaddr_t topstack = stackptr;
  for (int i = 0; i <= argc; ++i) {
    result = copyout((const void *)&argptr[i], (userptr_t)stackptr, sizeof(char *));
    if(result)  return result;
    stackptr += sizeof(char *);
  }
  stackptr = topstack;

  /* 6. Free stuff that's used */
  // destroy old addrspace
  as_destroy(oldas);
  // free every arg copied to the kernel
  for (int i = 0; i < argc; ++i) { kfree(kargs[i]); }

  // kfree(kprog_temp);
  /* Warp to user mode. */
  // (userptr_t)args
  enter_new_process(argc /*argc*/, (userptr_t)stackptr /*userspace addr of argv*/,
                    stackptr, entrypoint);

  /* enter_new_process does not return. */
  panic("enter_new_process returned\n");
  return EINVAL;
}

/* sys_fork handler */
int sys_fork(struct trapframe *tf, pid_t *retval) {
  struct proc *myproc = curproc;
  int ret = 0;
  // create child process struct
  struct proc *child = proc_create_runprogram("my child");
  if(child == NULL) return ENOMEM;

  // addr space
  spinlock_acquire(&child->p_lock);
  ret = as_copy(myproc->p_addrspace, &child->p_addrspace);
  if(ret) {
    spinlock_release(&child->p_lock);
    return ret;  
  }

  /* child->pid already assigned in proc_create() */
  child->parent = myproc;
  struct pinfo *pfull = kmalloc(sizeof(struct pinfo));
  // pfull->pc = child;
  pfull->pid = child->pid;
  pfull->exitcode = -10;    // child still running
  ret = array_add(myproc->children, pfull, NULL);
  if(ret) {
    spinlock_release(&child->p_lock);
    return ret;
  }

  //create a thread for child
  // spinlock_acquire(&child->p_lock);
  struct trapframe *mytf = kmalloc(sizeof(struct trapframe));
  if(mytf == NULL) {
    spinlock_release(&child->p_lock);
    return ENOMEM;
  }
  memcpy((void *)mytf, tf, sizeof(struct trapframe));
  spinlock_release(&child->p_lock);
  ret = thread_fork("child_thread", child, (void *)&enter_forked_process, 
                                                  (void *)mytf, 3); 
  if(ret) return ret;

  // Success
  *retval = child->pid;
  return 0; 
}
#endif /* OPT_A2 */

  /* this implementation of sys__exit does not do anything with the exit code */
  /* this needs to be fixed to get exit() and waitpid() working properly */

void sys__exit(int exitcode) {
  struct addrspace *as;
  struct proc *p = curproc;
  /* for now, just include this to keep the compiler from complaining about
     an unused variable */
  // (void)exitcode;

#if OPT_A2
  // parent is still running. Add exitcode info to deadkids
  if(p->parent && p->parent->ecode == -10) {
    lock_acquire(p->parent->lk);
    /* update exitcode for child pinfo */
    for (unsigned int i = 0; i < p->parent->children->num; ++i) {
      struct pinfo *kidfull = (struct pinfo *)array_get(p->parent->children, i);
      if(p->pid == kidfull->pid) {
        kidfull->exitcode = exitcode; // @param exitcode >= 0
        break;
      }
    }
    // this proc is exiting, notify waitpid()
    cv_signal(p->parent->runningChild, p->parent->lk); // p->runningChild
    lock_release(p->parent->lk);
  }
#endif // OPT_A2

  DEBUG(DB_SYSCALL,"Syscall: _exit(%d)\n",exitcode);
  KASSERT(curproc->p_addrspace != NULL);
  as_deactivate();
  /*
    * clear p_addrspace before calling as_destroy. Otherwise if
    * as_destroy sleeps (which is quite possible) when we
    * come back we'll be calling as_activate on a
    * half-destroyed address space. This tends to be
    * messily fatal.
    */
  as = curproc_setas(NULL);
  as_destroy(as);
  /* detach this thread from its process */
  /* note: curproc cannot be used after this call */  
  proc_remthread(curthread);
  /* if this is the last user process in the system, proc_destroy()
     will wake up the kernel menu thread */
  proc_destroy(p);
  thread_exit();
  /* thread_exit() does not return, so we should never get here */
  panic("return from thread_exit in sys_exit\n");
}


/* stub handler for getpid() system call                */
int
sys_getpid(pid_t *retval)
{
  /* for now, this is just a stub that always returns a PID of 1 */
  /* you need to fix this to make it work properly */
#if OPT_A2
  *retval = curproc->pid;
#endif // OPT_A2
  return(0);
}

/* stub handler for waitpid() system call                */

int
sys_waitpid(pid_t pid,
	    userptr_t status,
	    int options,
	    pid_t *retval)
{
  int exitstatus;
  int result;

  /* this is just a stub implementation that always reports an
     exit status of 0, regardless of the actual exit status of
     the specified process.   
     In fact, this will return 0 even if the specified process
     is still running, and even if it never existed in the first place.

     Fix this!
  */

  if (options != 0) {
    return(EINVAL);
  }

#if OPT_A2
  // status is invalid ptr
  if(status == NULL)  return EFAULT;

  struct proc *pa = curproc;
  lock_acquire(pa->lk);
  bool found = false;
  struct pinfo *kidfull = NULL;
  // check if pid is a child of curproc
  for (unsigned int i = 0; i < pa->children->num; ++i) {
    kidfull = (struct pinfo *)array_get(pa->children, i);
    if(pid == kidfull->pid) {
      found = true;
      // child proc is still running
      while(kidfull->exitcode == -10) { 
        cv_wait(pa->runningChild, pa->lk); // kidfull->pc->runningChild
      }
      // child proc has/just exited
      exitstatus = _MKWAIT_EXIT(kidfull->exitcode);
      /* Do NOT allow waitpid() called more than once on the same pid */
      kfree(kidfull);
      kidfull = NULL;
      array_remove(pa->children, i);
      /* Changes */
      break;
    }
  } 
  //pid not one of curproc's children
  if(!found) {
    lock_release(pa->lk);
    return ECHILD;  
  }
  lock_release(pa->lk);
#endif // OPT_A2

  /* for now, just pretend the exitstatus is 0 */
  // exitstatus = 0;
  result = copyout((void *)&exitstatus,status,sizeof(int));
  if (result) {
    return(result);
  }
  *retval = pid;
  return(0);
}




