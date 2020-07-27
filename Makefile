nproc=$(shell python3 -c 'import multiprocessing; print( max(multiprocessing.cpu_count() - 1, 1))')

CC ?= clang
CXX ?= clang++

.PHONY: plugin.dbg.so
plugin.dbg.so:
	mkdir -p build/ && \
	cd build/ && \
	cmake .. -DCMAKE_C_COMPILER=$(CC) -DCMAKE_CXX_COMPILER=$(CXX) && \
	make "-j$(nproc)" && \
	cd .. && \
	rm -f  plugin.dbg.so && \
	ln -s build/ov_msckf/libslam2.so plugin.dbg.so && \
	true

.PHONY: plugin.opt.so
plugin.opt.so:
	mkdir -p build/ && \
	cd build/ && \
	cmake -DDNDEBUG=1 -DCMAKE_BUILD_TYPE=Release .. && \
	make "-j$(nproc)" && \
	cd .. && \
	rm -f  plugin.opt.so && \
	ln -s build/ov_msckf/libslam2.so plugin.opt.so && \
	true

.PHONY: clean
clean:
	touch build && rm -rf build *.so
