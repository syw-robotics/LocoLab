"""Readable policy YAML: commented sections and compact, wrapped joint arrays."""

from pathlib import Path
import textwrap

import yaml

from . import TEMPLATE_DIR


class _Dumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True

    def increase_indent(self, flow=False, indentless=False):
        return super().increase_indent(flow, False)


class _QuotedString(str):
    pass


def _list(dumper, values):
    return dumper.represent_sequence("tag:yaml.org,2002:seq", values,
                                   flow_style=all(not isinstance(v, (dict, list)) for v in values))


_Dumper.add_representer(list, _list)
_Dumper.add_representer(
    _QuotedString, lambda dumper, value: dumper.represent_scalar("tag:yaml.org,2002:str", value, style='"')
)


def write_policy_yaml(path: str | Path, config: dict) -> None:
    layout = yaml.safe_load((TEMPLATE_DIR / "policy_layout.yaml").read_text(encoding="utf-8"))
    sections = [f"# {layout['header']}\n"]
    written = set()
    for section in layout["sections"]:
        values = {key: config[key] for key in section["fields"] if key in config}
        if not values:
            continue
        written.update(values)
        lines = []
        for key, value in values.items():
            if key.endswith("_joint_order") or key in (
                "robot", "imu_source", "policy_path", "root_body_name", "anchor_body_name", "body_names",
            ):
                value = [_QuotedString(name) for name in value] if isinstance(value, list) else _QuotedString(value)
            comment = section["fields"][key]
            if comment:
                lines.append(f"# {comment}\n")
            if isinstance(value, list) and len(value) > 8 and not any(isinstance(v, (dict, list)) for v in value):
                # Keep long joint arrays compact without PyYAML's one-value-per-line output.
                items = yaml.dump(value, Dumper=_Dumper, width=100000).strip()[1:-1]
                lines.append(f"{key}: [\n{textwrap.fill(items, width=110, initial_indent='  ', subsequent_indent='  ')}\n]\n")
            else:
                lines.append(yaml.dump({key: value}, Dumper=_Dumper, sort_keys=False, width=110))
        sections.append(f"# {'-' * 72}\n# {section['title']}\n# {section['comment']}\n" + "".join(lines))
    missing = set(config) - written
    if missing:
        raise ValueError(f"Undocumented policy YAML fields: {sorted(missing)}")
    Path(path).write_text("\n".join(sections), encoding="utf-8")
