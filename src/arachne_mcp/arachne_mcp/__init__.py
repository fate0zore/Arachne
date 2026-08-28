"""Arachne MCP exposure package.

Two independent MCP servers (Streamable HTTP), split by safety boundary:

- ``viewpoint_planner_mcp`` (read-only): exposes viewpoint planning tools
  backed by the ``/arachne/viewpoint/plan`` service.
- ``viewpoint_acquire_mcp`` (motion, gated): exposes the viewpoint acquire
  chain (move to viewpoint -> capture -> persist) reusing the headless
  ``TeachPanelNode`` motion/capture core.

See docs/todo/mcp_exposure_todo_plan.md for the full design.
"""
