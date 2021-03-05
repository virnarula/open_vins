#pragma once

extern void* __hpvm__launch(void*, ...);
extern void __hpvm__wait(void*);
extern void* __hpvm_parallel_section_begin();
extern void __hpvm_parallel_section_end(void*);
extern void* __hpvm_task_begin();
extern void* __hpvm_task_begin(unsigned,...);
extern void __hpvm_task_end(void*);
extern void __hpvm_parallel_loop(unsigned);
extern void __hpvm_parallel_loop(unsigned, ...);

