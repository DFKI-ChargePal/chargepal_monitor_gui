import marimo

__generated_with = "0.8.17"
app = marimo.App(width="medium")


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
    refresh_button = mo.ui.refresh(label="Refresh", default_interval=None, options=["1s", "2s", "5s"])
    refresh_button

    return refresh_button,


@app.cell
def __(cfg, ldb, mo, refresh_button, robot_info):
    refresh_button
    _robot_info_df = mo.sql(
        f"""
        INSTALL sqlite;
        LOAD sqlite;
        ATTACH '{cfg.ldb_path}' AS ldb (TYPE SQLITE);
        SELECT * FROM ldb.main.robot_info;
        """
    )
    return ldb,


@app.cell
def __(mo):
    mo.md(r"""# &nbsp;""")
    return


@app.cell
def __(add_form):
    #mo.vstack([add_form, add_form.value])
    add_form
    return


@app.cell
def __(add_robot_button):
    add_robot_button
    return


@app.cell
def __(add_form, add_robot_button, mo, update_robot):
    mo.stop(not add_robot_button.value)
    response = None
    if add_form.value:
        response = update_robot(add_form.value)
        mo.output.clear()
    response if response is not None else None
    return response,


@app.cell
def __(mo):
    mo.md("""# &nbsp;""")
    return


@app.cell
def __(delete_form):
    #mo.vstack([delete_form, delete_form.value['robot_dropdown']])
    delete_form
    return


@app.cell
def __(delete_robot_button):
    delete_robot_button
    return


@app.cell
def __(delete_form, delete_robot_button, mo):
    mo.stop(not delete_robot_button.value)
    _name = delete_form.value['robot_dropdown']
    _df = mo.sql(
        f"""
        DELETE FROM ldb.main.robot_info
        WHERE name='{_name}';
        """
    )
    return


@app.cell
def __(mo):
    add_form = mo.vstack(
        [
            mo.md("""
                ## Add new robot or update existing one
                Robot Name:
                {new_robot_name}
                Robot Location:
                {new_robot_location}
            """),
            mo.accordion(
                {
                    'Extended Properties': mo.md("""Robot Availability:
                                        {new_robot_availability}
                                        Robot Charge:
                                        {new_robot_charge}
                                        Robot Error Count:
                                        {new_robot_error_count}""")
                }
            )
        ]
    ).batch(
        new_robot_name=mo.ui.text(placeholder='Robot name...', full_width=True),
        new_robot_location=mo.ui.text(placeholder='Robot location...', full_width=True),
        new_robot_availability=mo.ui.text(
            placeholder='Defaults to 1 for new robots', full_width=True
        ),
        new_robot_charge=mo.ui.text(
            placeholder='Defaults to 100 for new robots', full_width=True
        ),
        new_robot_error_count=mo.ui.text(
            placeholder='Defaults to 0 for new robots', full_width=True
        ),
    )
    return add_form,


@app.cell
def __():
    def check_and_add(name, value, values):
        if value != '':
            if values == '':
                values = f"{name} = '{value}'"
            else:
                values = values + f", {name} = '{value}'"
        return values
    return check_and_add,


@app.cell
def __(check_and_add, ldb, mo, robot_info):
    def update_robot(robot_values):
        robot_name = robot_values['new_robot_name']
        location = robot_values['new_robot_location']
        availability = robot_values['new_robot_availability']
        robot_charge = robot_values['new_robot_charge']
        error_count = robot_values['new_robot_error_count']
        response = None
        if robot_name != '':
            _df = mo.sql(
                f"""
                SELECT name
                FROM ldb.main.robot_info
                WHERE name='{robot_name}';
                """
            )
            if type(_df) == type(None):
                # Add new entry:
                if location != '':
                    availability = 1 if availability == '' else availability
                    robot_charge = 100.0 if robot_charge == '' else robot_charge
                    error_count = 0 if error_count == '' else error_count
                    _ = mo.sql(
                        f"""
                        INSERT INTO ldb.main.robot_info (name, robot_location, availability, robot_charge, error_count)
                        VALUES ('{robot_name}', '{location}', '{availability}', '{robot_charge}', '{error_count}');
                        """
                    )
                else:
                    response = mo.callout(
                        'A location is needed for creating a new entry!', kind='alert'
                    )
            else:
                # Update existing entry:
                set_values = ""
                for name, value in zip(
                    ['robot_location', 'availability', 'robot_charge', 'error_count'],
                    [location, availability, robot_charge, error_count],
                ):
                    set_values = check_and_add(name, value, set_values)
                _ = mo.sql(
                    f"""
                    UPDATE ldb.main.robot_info
                    SET {set_values}
                    WHERE name = '{robot_name}';
                    """
                )
        return response if response is not None else ''
    return update_robot,


@app.cell
def __(mo):
    add_robot_button = mo.ui.run_button(label='Add/Update Robot')
    return add_robot_button,


@app.cell
def __(mo):
    delete_robot_button = mo.ui.run_button(label='Delete Robot')
    return delete_robot_button,


@app.cell
def __(ldb, mo, robot_info):
    _robots = mo.sql(
        f"""
        SELECT name
        FROM ldb.main.robot_info;
        """
    )
    delete_form = mo.md(
        """
        ## Delete existing robot
        {robot_dropdown}
        """
    ).batch(
        robot_dropdown=mo.ui.dropdown.from_series(_robots["name"], label='Select Robot:')
    )
    mo.output.clear()
    return delete_form,


@app.cell
def __(cfg):
    cfg.gui_sidebar
    return


if __name__ == "__main__":
    app.run()
