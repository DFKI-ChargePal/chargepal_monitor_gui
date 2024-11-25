import marimo

__generated_with = "0.9.21"
app = marimo.App(width="medium")


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
    mo.md(r"""# Set Environment""")
    return


@app.cell
def __(mo):
    mo.md(r"""This form can be used to update the environment.""")
    return


@app.cell
def __(mo):
    mo.md(r"""# Robot""")
    return


@app.cell
def __(add_form_robot):
    #mo.vstack([add_form_robot, add_form_robot.value])
    add_form_robot
    return


@app.cell
def __(add_robot_button):
    add_robot_button
    return


@app.cell
def __(mo):
    mo.md("""# &nbsp;""")
    return


@app.cell
def __(mo, refresh_robot_dd_button, robot_delete_dd):
    mo.vstack([mo.md("## Delete existing robot"), mo.hstack([robot_delete_dd, refresh_robot_dd_button], justify="start")])
    return


@app.cell
def __(delete_robot_button):
    delete_robot_button
    return


@app.cell
def __(mo):
    mo.md(r"""# Cart""")
    return


@app.cell
def __(add_form_cart):
    add_form_cart
    return


@app.cell
def __(add_cart_button):
    add_cart_button
    return


@app.cell
def __(mo):
    mo.md(r"""# &nbsp;""")
    return


@app.cell
def __(cart_delete_dd, mo, refresh_cart_dd_button):
    mo.vstack([mo.md("## Delete existing cart"), mo.hstack([cart_delete_dd, refresh_cart_dd_button], justify="start")])
    return


@app.cell
def __(delete_cart_button):
    delete_cart_button
    return


@app.cell
def __():
    # General code below:
    return


@app.cell
def __():
    def check_and_add(name, value, values):
        if value != '':
            if values == '':
                values = f"{name} = '{value}'"
            else:
                values = values + f", {name} = '{value}'"
        return values
    return (check_and_add,)


@app.cell
def __(ldb, mo, null_info):
    def get_entries(robot_or_cart):
        entries = mo.sql(
            f"""
            SELECT name
            FROM ldb.main.{robot_or_cart}_info;
            """
        )
        return entries
    return (get_entries,)


@app.cell
def __(cfg):
    cfg.gui_sidebar
    return


@app.cell
def __():
    # Robot code below:
    return


@app.cell
def __(mo):
    add_robot_button = mo.ui.run_button(label='Add/Update Robot')
    delete_robot_button = mo.ui.run_button(label='Delete Robot')
    refresh_robot_dd_button = mo.ui.refresh()
    mo.output.clear()
    return add_robot_button, delete_robot_button, refresh_robot_dd_button


@app.cell
def __(get_entries, mo, refresh_robot_dd_button):
    refresh_robot_dd_button
    robot_delete_dd = mo.ui.dropdown.from_series(get_entries("robot")["name"], label='Select Robot:')
    mo.output.clear()
    return (robot_delete_dd,)


@app.cell
def __(add_form_robot, add_robot_button, mo, update_robot):
    mo.stop(not add_robot_button.value)
    robot_response = None
    if add_form_robot.value:
        robot_response = update_robot(add_form_robot.value)
        mo.output.clear()
    robot_response if robot_response is not None else None
    return (robot_response,)


@app.cell
def __(delete_robot_button, ldb, mo, robot_delete_dd, robot_info):
    mo.stop(not delete_robot_button.value)
    _name = robot_delete_dd.value
    _df = mo.sql(
        f"""
        DELETE FROM ldb.main.robot_info
        WHERE name='{_name}';
        """
    )
    return


@app.cell
def __(mo):
    add_form_robot = mo.vstack(
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
    return (add_form_robot,)


@app.cell
def __(check_and_add, ldb, mo, robot_info):
    def update_robot(robot_values):
        robot_name = robot_values['new_robot_name']
        location = robot_values['new_robot_location']
        availability = robot_values['new_robot_availability']
        robot_charge = robot_values['new_robot_charge']
        error_count = robot_values['new_robot_error_count']
        robot_response = None
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
                    robot_response = mo.callout(
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
        return robot_response if robot_response is not None else ''
    return (update_robot,)


@app.cell
def __():
    # Cart code below:
    return


@app.cell
def __(mo):
    add_cart_button = mo.ui.run_button(label='Add/Update Cart')
    delete_cart_button = mo.ui.run_button(label='Delete Cart')
    refresh_cart_dd_button = mo.ui.refresh()
    mo.output.clear()
    return add_cart_button, delete_cart_button, refresh_cart_dd_button


@app.cell
def __(get_entries, mo, refresh_cart_dd_button):
    refresh_cart_dd_button
    cart_delete_dd = mo.ui.dropdown.from_series(get_entries("cart")["name"], label='Select Cart:')
    mo.output.clear()
    return (cart_delete_dd,)


@app.cell
def __(add_cart_button, add_form_cart, mo, update_cart):
    mo.stop(not add_cart_button.value)
    cart_response = None
    if add_form_cart.value:
        cart_response = update_cart(add_form_cart.value)
        mo.output.clear()
    cart_response if cart_response is not None else None
    return (cart_response,)


@app.cell
def __(cart_delete_dd, cart_info, delete_cart_button, ldb, mo):
    mo.stop(not delete_cart_button.value)
    _name = cart_delete_dd.value
    _df = mo.sql(
        f"""
        DELETE FROM ldb.main.cart_info
        WHERE name='{_name}';
        """
    )
    return


@app.cell
def __(mo):
    add_form_cart = mo.vstack(
        [
            mo.md("""
                ## Add new cart or update existing one
                Cart Name:
                {new_cart_name}
                Cart Location:
                {new_cart_location}
            """),
            mo.accordion(
                {
                    'Extended Properties': mo.md("""Cart Plugged State:
                                        {new_cart_plugged}
                                        Cart Error Count:
                                        {new_cart_error_count}""")
                }
            )
        ]
    ).batch(
        new_cart_name=mo.ui.text(placeholder='Cart name...', full_width=True),
        new_cart_location=mo.ui.text(placeholder='Cart location...', full_width=True),
        new_cart_plugged=mo.ui.text(
            placeholder='Defaults to 0 for new carts', full_width=True
        ),
        new_cart_error_count=mo.ui.text(
            placeholder='Defaults to 0 for new carts', full_width=True
        ),
    )
    return (add_form_cart,)


@app.cell
def __(cart_info, check_and_add, ldb, mo):
    def update_cart(cart_values):
        cart_name = cart_values['new_cart_name']
        location = cart_values['new_cart_location']
        cart_plugged = cart_values['new_cart_plugged']
        error_count = cart_values['new_cart_error_count']
        cart_response = None
        if cart_name != '':
            _df = mo.sql(
                f"""
                SELECT name
                FROM ldb.main.cart_info
                WHERE name='{cart_name}';
                """
            )
            if type(_df) == type(None):
                # Add new entry:
                if location != '':
                    cart_plugged = 0 if cart_plugged == '' else cart_plugged
                    error_count = 0 if error_count == '' else error_count
                    _ = mo.sql(
                        f"""
                        INSERT INTO ldb.main.cart_info (name, cart_location, plugged, error_count)
                        VALUES ('{cart_name}', '{location}', '{cart_plugged}', '{error_count}');
                        """
                    )
                else:
                    cart_response = mo.callout(
                        'A location is needed for creating a new entry!', kind='alert'
                    )
            else:
                # Update existing entry:
                set_values = ""
                for name, value in zip(
                    ['cart_location', 'plugged', 'error_count'],
                    [location, cart_plugged, error_count],
                ):
                    set_values = check_and_add(name, value, set_values)
                _ = mo.sql(
                    f"""
                    UPDATE ldb.main.cart_info
                    SET {set_values}
                    WHERE name = '{cart_name}';
                    """
                )
        return cart_response if cart_response is not None else ''
    return (update_cart,)


if __name__ == "__main__":
    app.run()
