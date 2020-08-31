nproc=$(shell python3 -c 'import multiprocessing; print( max(multiprocessing.cpu_count() - 1, 1))')

CXX := clang++-10
CC := clang-10

.PHONY: plugin.dbg.so
plugin.dbg.so: build/Debug/Makefile
	make -C build/Debug "-j$(nproc)" && \
	rm -f $@ && \
	ln -s build/Debug/ov_msckf/libslam2.so $@ && \
	true

.PHONY: plugin.opt.so
plugin.opt.so: build/Release/Makefile
	make -C build/Release "-j$(nproc)" && \
	rm -f $@ && \
	ln -s build/Release/ov_msckf/libslam2.so $@ && \
	true

build/%/Makefile:
	mkdir -p build/Debug && \
	cd build/Debug && \
	cmake -DCMAKE_BUILD_TYPE=Debug  -DCMAKE_CXX_COMPILER=$(CXX) -DCMAKE_C_COMPILER=$(CC) ../.. && \
	true

build/Release/Makefile:
	mkdir -p build/Release && \
	cd build/Release && \
	cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=$(CXX) -DCMAKE_C_COMPILER=$(CC) ../.. && \
	true

.PHONY: clean
clean:
	touch build && rm -rf build *.so
