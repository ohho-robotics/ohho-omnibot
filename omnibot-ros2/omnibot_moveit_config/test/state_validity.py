# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
"""Format a Jazzy moveit_msgs/srv/GetStateValidity response.

The response fields are valid, contacts, cost_sources, and
constraint_result. There is no error_code.
"""


def failure_detail(response):
    contacts = [
        f"{contact.contact_body_1}/{contact.contact_body_2}"
        for contact in response.contacts
    ]
    constraints = [
        f"result={item.result} distance={item.distance}"
        for item in response.constraint_result
    ]
    costs = [source.cost_density for source in response.cost_sources]
    return (
        f"valid={bool(response.valid)} contacts={contacts} "
        f"constraint_result={constraints} cost_sources={costs}"
    )
