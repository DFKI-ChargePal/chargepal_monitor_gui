import marimo

__generated_with = '0.8.17'
app = marimo.App(width='medium')


@app.cell
def __():
    import os

    import config as cfg
    import marimo as mo

    return cfg, mo, os


# Uncomment the following lines for standalone execution of this notebook:
# @app.cell
# def __(cfg, mo):
#     mo.sql(
#         f"""
#         INSTALL sqlite;
#         LOAD sqlite;
#         ATTACH '{cfg.ldb_path}' AS ldb (TYPE SQLITE);
#         ATTACH '{cfg.pdb_path}' AS pdb (TYPE SQLITE);
#         """
#     )
#     return ldb, pdb


@app.cell
def __(mo):
    def get_table(table_name: str):
        try:
            df = mo.sql(
                f"""
                USE ldb;
                SELECT * FROM ldb.main.{table_name};
                """
            )
            table = mo.ui.table(
                data=df,
                freeze_columns_left=[df.columns[0]],
                selection=None,
                show_column_summaries=False,
            )
        except (TypeError, AttributeError):
            table = mo.ui.text(
                f'The {table_name} table is currently empty.', full_width=True
            )
        return table

    return (get_table,)


@app.cell
def __(mo):
    mo.md("""# Local Database""")
    return


@app.cell
def __(mo):
    refresh_button = mo.ui.refresh()
    refresh_button

    return refresh_button,


@app.cell
def __(get_table, mo, refresh_button):
    refresh_button
    mo.ui.tabs(
        {
            'Robots': get_table('robot_info'),
            'Carts': get_table('cart_info'),
            'Environment': get_table('env_info'),
            'Orders': get_table('orders_in'),
        }
    )
    return


@app.cell
def __(cfg):
    cfg.gui_sidebar
    return


if __name__ == '__main__':
    app.run()
