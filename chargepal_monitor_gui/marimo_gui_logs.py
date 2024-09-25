import marimo

__generated_with = '0.8.17'
app = marimo.App(width='medium')


@app.cell
def __():
    import os

    import config as cfg
    import marimo as mo

    return cfg, mo, os


@app.cell
def __(mo):
    mo.md(r"""# Logs""")
    return


@app.cell
def __(mo):
    refresh_button = mo.ui.refresh(options=["5s", "10s", "30s"])
    refresh_button

    return refresh_button,


@app.cell
def __(cfg, mo, os, refresh_button):
    refresh_button
    log_files = {}
    for file in os.listdir(cfg.logs_path):
        basename, extension = os.path.splitext(file)
        file_path = os.path.join(cfg.logs_path, file)
        if os.path.isfile(file_path):
            log_files[file] = '\n'.join(reversed(list(open(file_path))))
    mo.ui.tabs(log_files)
    return basename, extension, file, file_path, log_files


@app.cell
def __(cfg):
    cfg.gui_sidebar
    return


if __name__ == '__main__':
    app.run()
