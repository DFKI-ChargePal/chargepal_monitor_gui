#!/usr/bin/env python3
from nicegui import ui, events
table_rows={
    "robot_rows": [],
    "cart_rows": [],
    "ads_rows": [],
    "bcs_rows": [],
    }

# Add a new row to the specified table
def add_row(table: ui.table, row_name: str) -> None:
    global table_rows
    new_id = max((r["id"] for r in table_rows[row_name]), default=-1) + 1
    table_rows[row_name].append({"id": new_id})
    table.update()
    ui.notify(f"Added new row with ID {new_id}")
    


# Rename a row in the specified table
def rename(e: events.GenericEventArguments, table: ui.table, row_name: str) -> None:
    global table_rows
    for row in table_rows[row_name]:
        if row["id"] == e.args["id"]:
            row.update(e.args)
    table.update()
    ui.notify(f"Updated row with ID {e.args['id']}")


# Delete a row from the specified table
def delete_row(e: events.GenericEventArguments, table: ui.table, row_name: str) -> None:
    global table_rows
    table_rows[row_name][:] = [r for r in table_rows[row_name] if r["id"] != e.args["id"]]
    table.update()
    ui.notify(f"Deleted row with ID {e.args['id']}")
    


# Create the GUI tables
def create_tables():
    robot_columns = [
        {
            "name": "robot_names",
            "label": "Robot",
            "field": "robot_names",
            "align": "center",
        },
        {
            "name": "rbs_names",
            "label": "Station",
            "field": "rbs_names",
            "align": "center",
        },
    ]

    cart_columns = [
        {"name": "cart_names", "label": "Cart", "field": "cart_names", "align": "center"},
        {
            "name": "bws_names",
            "label": "Station",
            "field": "bws_names",
            "align": "center",
        },
    ]

    ads_columns = [
        {"name": "ads_names", "label": "ADS_Station", "field": "ads_names", "align": "center"},
    ]

    bcs_columns = [
        {"name": "bcs_names", "label": "BCS_Station", "field": "bcs_names", "align": "center"},
    ]

    # Define the tables with columns and rows
    robot_table = ui.table(
        columns=robot_columns,
        rows=table_rows["robot_rows"],
        row_key="robot_names",
        title="Robot details",
    ).classes("w-80")
    cart_table = ui.table(
        columns=cart_columns, rows=table_rows["cart_rows"], row_key="cart_names", title="Cart details"
    ).classes("w-80")
    ads_table = ui.table(
        columns=ads_columns, rows=table_rows["ads_rows"], row_key="ads", title="ADS Station details"
    ).classes("w-80")
    bcs_table = ui.table(
        columns=bcs_columns, rows=table_rows["bcs_rows"], row_key="bcs", title="BCS Station details"
    ).classes("w-80")

    return {
        "robot_table": robot_table,
        "cart_table": cart_table,
        "ads_table": ads_table,
        "bcs_table": bcs_table,
    }

def add_env_table_slots(tables: ui.table):
    tables["robot_table"].add_slot(
            "header",
            r"""
            <q-tr :props="props">
                <q-th auto-width />
                <q-th v-for="col in props.cols" :key="col.name" :props="props">
                    {{ col.label }}
                </q-th>
            </q-tr>
        """,
        )

    tables["ads_table"].add_slot(
        "header",
        r"""
        <q-tr :props="props">
            <q-th auto-width />
            <q-th v-for="col in props.cols" :key="col.name" :props="props">
                {{ col.label }}
            </q-th>
        </q-tr>
    """,
    )

    tables["bcs_table"].add_slot(
        "header",
        r"""
        <q-tr :props="props">
            <q-th auto-width />
            <q-th v-for="col in props.cols" :key="col.name" :props="props">
                {{ col.label }}
            </q-th>
        </q-tr>
    """,
    )

    tables["cart_table"].add_slot(
        "header",
        r"""
        <q-tr :props="props">
            <q-th auto-width />
            <q-th v-for="col in props.cols" :key="col.name" :props="props">
                {{ col.label }}
            </q-th>
        </q-tr>
    """,
    )
    tables["robot_table"].add_slot(
        "body",
        r"""
        <q-tr :props="props">
            <q-td auto-width >
                <q-btn size="sm" color="warning" round dense icon="delete"
                    @click="() => $parent.$emit('delete', props.row)"
                />
            </q-td>
            <q-td key="robot_names" :props="props">
                {{ props.row.robot_names }}
                <q-popup-edit v-model="props.row.robot_names" v-slot="scope"
                    @update:model-value="() => $parent.$emit('rename', props.row)"
                >
                    <q-input v-model="scope.value" dense autofocus counter @keyup.enter="scope.set" />
                </q-popup-edit>
            </q-td>
            <q-td key="rbs_names" :props="props">
                {{ props.row.rbs_names }}
                <q-popup-edit v-model="props.row.rbs_names" v-slot="scope"
                    @update:model-value="() => $parent.$emit('rename', props.row)"
                >
                    <q-input v-model="scope.value" dense autofocus counter @keyup.enter="scope.set" />
                </q-popup-edit>
            </q-td>
        </q-tr>

    """,
    )

    tables["ads_table"].add_slot(
        "body",
        r"""
        <!-- Adapter Station details -->
        <q-tr :props="props">
            <q-td auto-width >
                <q-btn size="sm" color="warning" round dense icon="delete"
                    @click="() => $parent.$emit('delete', props.row)"
                />
            </q-td>
            <q-td key="ads_names" :props="props">
                {{ props.row.ads_names }}
                <q-popup-edit v-model="props.row.ads_names" v-slot="scope"
                    @update:model-value="() => $parent.$emit('rename', props.row)"
                >
                    <q-input v-model="scope.value" dense autofocus counter @keyup.enter="scope.set" />
                </q-popup-edit>
            </q-td>
        </q-tr>
    """,
    )

    tables["bcs_table"].add_slot(
        "body",
        r"""
        <q-tr :props="props">
            <q-td auto-width >
                <q-btn size="sm" color="warning" round dense icon="delete"
                    @click="() => $parent.$emit('delete', props.row)"
                />
            </q-td>
            <q-td key="bcs_names" :props="props">
                {{ props.row.bcs_names }}
                <q-popup-edit v-model="props.row.bcs_names" v-slot="scope"
                    @update:model-value="() => $parent.$emit('rename', props.row)"
                >
                    <q-input v-model="scope.value" dense autofocus counter @keyup.enter="scope.set" />
                </q-popup-edit>
            </q-td>
        </q-tr>
    """,
    )
    tables["cart_table"].add_slot(
        "body",
        r"""
        <q-tr :props="props">
            <q-td auto-width >
                <q-btn size="sm" color="warning" round dense icon="delete"
                    @click="() => $parent.$emit('delete', props.row)"
                />
            </q-td>
            <q-td key="cart_names" :props="props">
                {{ props.row.cart_names }}
                <q-popup-edit v-model="props.row.cart_names" v-slot="scope"
                    @update:model-value="() => $parent.$emit('rename', props.row)"
                >
                    <q-input v-model="scope.value" dense autofocus counter @keyup.enter="scope.set" />
                </q-popup-edit>
            </q-td>
            <q-td key="bws_names" :props="props">
                {{ props.row.bws_names }}
                <q-popup-edit v-model="props.row.bws_names" v-slot="scope"
                    @update:model-value="() => $parent.$emit('rename', props.row)"
                >
                    <q-input v-model="scope.value" dense autofocus counter @keyup.enter="scope.set" />
                </q-popup-edit>
            </q-td>
        </q-tr>

    """,
    )

    # Configure table events
    with tables["robot_table"].add_slot("bottom-row"):
        with tables["robot_table"].cell().props("colspan=3"):
            ui.button(
                "Add row",
                icon="add",
                color="accent",
                on_click=lambda e : add_row(tables["robot_table"],"robot_rows"),
            ).classes("w-full")
    tables["robot_table"].on(
        "rename", lambda e: rename(e, tables["robot_table"], "robot_rows")
    )
    tables["robot_table"].on(
        "delete", lambda e: delete_row(e, tables["robot_table"], "robot_rows")
    )
    
    with tables["ads_table"].add_slot("bottom-row"):
        with tables["robot_table"].cell().props("colspan=3"):
            ui.button(
                "Add row",
                icon="add",
                color="accent",
                on_click=lambda e : add_row(tables["ads_table"],"ads_rows"),
            ).classes("w-full")
    tables["ads_table"].on("rename", lambda e: rename(e, tables["ads_table"], "ads_rows"))
    tables["ads_table"].on(
        "delete", lambda e: delete_row(e, tables["ads_table"], "ads_rows")
    )
    
    with tables["bcs_table"].add_slot("bottom-row"):
        with tables["bcs_table"].cell().props("colspan=3"):
            ui.button(
                "Add row",
                icon="add",
                color="accent",
                on_click=lambda e : add_row(tables["bcs_table"],"bcs_rows"),
            ).classes("w-full")
    tables["bcs_table"].on("rename", lambda e: rename(e, tables["bcs_table"], "bcs_rows"))
    tables["bcs_table"].on(
        "delete", lambda e: delete_row(e, tables["bcs_table"], "bcs_rows")
    )
    
    with tables["cart_table"].add_slot("bottom-row"):
        with tables["cart_table"].cell().props("colspan=3"):
            ui.button(
                "Add row",
                icon="add",
                color="accent",
                on_click=lambda e : add_row(tables["cart_table"],"cart_rows"),
            ).classes("w-full")
    tables["cart_table"].on("rename", lambda e: rename(e, tables["cart_table"], "cart_rows"))
    tables["cart_table"].on(
        "delete", lambda e: delete_row(e, tables["cart_table"], "cart_rows")
    )
