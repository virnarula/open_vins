#pragma once

extern void* __hpvm_launch_begin(unsigned, ...) noexcept;
extern void __hpvm_launch_end(void*) noexcept;
extern void* __hpvm__launch(void*, ...) noexcept;
extern void __hpvm__wait(void*) noexcept;
extern void* __hpvm_parallel_section_begin() noexcept;
extern void __hpvm_parallel_section_end(void*) noexcept;
extern void* __hpvm_task_begin() noexcept;
extern void* __hpvm_task_begin(unsigned,...) noexcept;
extern void __hpvm_task_end(void*) noexcept;
extern void __hpvm_parallel_loop(unsigned) noexcept;
extern void __hpvm_parallel_loop(unsigned, ...) noexcept;

