nproc=$(shell python3 -c 'import multiprocessing; print( max(multiprocessing.cpu_count() - 1, 1))')
use_integ=$(ILLIXR_INTEGRATION)

CXX := clang++-10
CC := clang-10

.PHONY: dbg
dbg: build/Debug/Makefile
	make -C build/Debug "-j$(nproc)" && \
	rm -f $@ && \
	ln -s build/Debug/ov_msckf/libslam2.so plugin.$@.so && \
	true

.PHONY: opt
opt: build/RelWithDebInfo/Makefile
	make -C build/RelWithDebInfo "-j$(nproc)" && \
	rm -f $@ && \
	ln -s build/RelWithDebInfo/ov_msckf/libslam2.so plugin.$@.so && \
	true

build/Debug/Makefile:
	mkdir -p build/Debug && \
	cd build/Debug && \
	echo "Using ILLIXR_INTEGRATION: $(use_integ)" && \
	cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_COMPILER=$(CXX) -DCMAKE_C_COMPILER=$(CC) -DILLIXR_INTEGRATION=$(use_integ) ../.. && \
	true

build/RelWithDebInfo/Makefile:
	mkdir -p build/RelWithDebInfo && \
	cd build/RelWithDebInfo && \
	echo "Using ILLIXR_INTEGRATION: $(use_integ)" && \
	cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_COMPILER=$(CXX) -DCMAKE_C_COMPILER=$(CC) -DILLIXR_INTEGRATION=$(use_integ) ../.. && \
	true

tests/run:
tests/gdb:

.PHONY: clean
clean:
	touch build && rm -rf build *.so
