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
    mo.md(r"""# Set Environment""")
    return


@app.cell
def __(mo):
    mo.md(r"""This is placeholder text. The environment setup will be added here.""")
    return


@app.cell
def __(cfg):
    cfg.gui_sidebar
    return


if __name__ == '__main__':
    app.run()
