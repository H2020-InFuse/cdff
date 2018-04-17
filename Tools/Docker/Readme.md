## Dockerfiles

This directory contains the `Dockerfiles` that describe:

* The Docker image used for the continuous integration (CI) of this Git repository on GitLab. The resulting image must be tagged `nexus.spaceapplications.com/repository/infuse/cdff-ci:<version>`:

    ```shell
    $ docker build --tag=nexus.spaceapplications.com/repository/infuse/cdff-ci:<version> /path/to/this/directory
    ```

* The Docker image that can be used by CDFF developers to:

    - Build and test the CDFF core and support components
    - Use the CDFF dev component (DFN and DFPC template generators)

    The resulting image must be tagged `h2020infuse/cdff:<version>` and `h2020infuse/cdff:latest`:

    ```shell
    $ docker build --tag=h2020infuse/cdff:<version> --tag=h2020infuse/cdff:latest --file=Dockerfile.user /path/to/this/directory
    ```

This directory also contains files that will be sent to the Docker daemon (the "context") so that they can be included into the resulting images.
