FROM opencv:4.5.5
COPY . .
RUN mkdir build \
    && cd build \
    && cmake .. \
    && make -j8