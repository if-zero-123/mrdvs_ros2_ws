import os

import uvicorn

from .api import create_app


def main() -> None:
    host = os.environ.get("MRDVS_WEB_HOST", "10.42.0.1")
    port = int(os.environ.get("MRDVS_WEB_PORT", "80"))
    uvicorn.run(create_app(), host=host, port=port, log_level="info")


if __name__ == "__main__":
    main()
