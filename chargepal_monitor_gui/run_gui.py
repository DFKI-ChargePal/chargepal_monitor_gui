from pathlib import Path

import config as cfg
import marimo
import uvicorn

env_gui_path = Path(__file__).parent / 'marimo_gui_env.py'
ldb_gui_path = Path(__file__).parent / 'marimo_gui_ldb.py'
pdb_gui_path = Path(__file__).parent / 'marimo_gui_pdb.py'
logs_gui_path = Path(__file__).parent / 'marimo_gui_logs.py'

marimo.sql(
    f"""
    INSTALL sqlite;
    LOAD sqlite;
    ATTACH '{cfg.ldb_path}' AS ldb (TYPE SQLITE);
    ATTACH '{cfg.pdb_path}' AS pdb (TYPE SQLITE);
    """
)

# Create a marimo asgi app
server = (
    marimo.create_asgi_app()
    .with_app(path='/', root=env_gui_path)
    .with_app(path='/ldb', root=ldb_gui_path)
    .with_app(path='/pdb', root=pdb_gui_path)
    .with_app(path='/logs', root=logs_gui_path)
)

# Run the server
if __name__ == '__main__':
    uvicorn.run(
        server.build(),
        host='0.0.0.0',
        port=cfg.gui_config['serve_port'],
    )
