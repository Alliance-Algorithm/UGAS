FROM opencv as build
COPY . .
RUN cmake . && make -j8 \
    && mv ugas ./UGAS/resources

FROM ubuntu
COPY --from=build ./usr ./usr
COPY --from=build ./UGAS/resources .
