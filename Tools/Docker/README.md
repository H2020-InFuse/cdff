## Memo for Docker image maintainers

This directory contains the `Dockerfiles` and the necessary build context required for creating:

* The Docker image used by the continuous integration (CI) pipeline of this Git repository on Space Applications' GitLab server. The resulting image must be tagged `nexus.spaceapplications.com/repository/infuse/cdff-ci:<version>` and must be uploaded to Space Applications' registry (Nexus):

    ```shell
    $ cd /path/to/this/directory
    $ docker build [--no-cache] --tag=nexus.spaceapplications.com/repository/infuse/cdff-ci:<version> ./
    ```

* The Docker image that can be used by CDFF developers to:

    - Build and test the CDFF core and support components
    - Use the CDFF dev component (DFN and DFPC template generators)

    The resulting image must be tagged `h2020infuse/cdff:<version>` and `h2020infuse/cdff:latest` and both tags must be pushed to the default registry (Docker Hub):

    ```shell
    $ cd /path/to/this/directory
    $ docker build [--no-cache] --tag=h2020infuse/cdff:<version> --tag=h2020infuse/cdff:latest --file=Dockerfile.user ./
    $ docker push h2020infuse/cdff:<version>
    $ docker push h2020infuse/cdff:latest
    ```

    The credentials for uploading to the default registry are as follow:

    ```shell
    $ docker login
    Username: h2020infuse
    Password: nfAGk9lQjG
    ```

    The command stores the encoded credentials in `$HOME/.docker/config.json` for future logins, so it's a one-time-only operation (that can be undone by `docker logout`). `HOME` is the home directory of `root` if you run the Docker client through `sudo -H|--set-home`, or it is the home directory of the current user if you don't use the `-H` option. Either way, the `.docker/` directory and the `config.json` file are created with `root:root` ownership. See also `docker-config-json(5)`.

All the files in this directory and recursive subdirectories are sent to the Docker daemon as the build context for inclusion into the resulting image according to the `COPY` instructions of the `Dockerfiles`.
