from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional
import xml.etree.ElementTree as ET

from flask import jsonify, request


def _ros2_src_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        candidate = parent / "SHOBOT_AMR_ws" / "src"
        if candidate.exists():
            return candidate
    return Path("SHOBOT_AMR_ws") / "src"


def _parse_package_xml(path: Path) -> Dict[str, str]:
    try:
        tree = ET.parse(path)
        root = tree.getroot()
    except ET.ParseError:
        return {"name": path.parent.name, "description": "Invalid package.xml", "version": ""}

    def _text(tag: str) -> str:
        node = root.find(tag)
        return node.text.strip() if node is not None and node.text else ""

    return {
        "name": _text("name") or path.parent.name,
        "version": _text("version"),
        "description": _text("description"),
    }


def _package_info(pkg_dir: Path) -> Dict[str, object]:
    package_xml = pkg_dir / "package.xml"
    info = _parse_package_xml(package_xml) if package_xml.exists() else {"name": pkg_dir.name}
    info["path"] = str(pkg_dir)
    launch_dir = pkg_dir / "launch"
    if launch_dir.exists():
        info["launch_files"] = sorted([p.name for p in launch_dir.iterdir() if p.is_file()])
    else:
        info["launch_files"] = []
    return info


def _package_launch_paths(pkg_dir: Path) -> List[str]:
    launch_dir = pkg_dir / "launch"
    if not launch_dir.exists():
        return []
    return sorted([str(p) for p in launch_dir.iterdir() if p.is_file()])


def _package_files(pkg_dir: Path, limit: int = 200) -> List[str]:
    files: List[str] = []
    for path in pkg_dir.rglob("*"):
        if path.is_file():
            files.append(str(path))
        if len(files) >= limit:
            break
    return files


def _find_package_dir(pkg_name: str) -> Optional[Path]:
    root = _ros2_src_root()
    direct = root / pkg_name
    if direct.exists():
        return direct
    for child in root.iterdir():
        if child.is_dir() and (child / "package.xml").exists():
            info = _parse_package_xml(child / "package.xml")
            if info.get("name") == pkg_name:
                return child
    return None


def register_ros2_package_routes(bp, store):
    @bp.route("/ros2/packages", methods=["GET"])
    def ros2_packages():
        root = _ros2_src_root()
        if not root.exists():
            return jsonify({"success": False, "message": "ROS2 workspace not found"}), 404

        packages: List[Dict[str, object]] = []
        for child in sorted(root.iterdir()):
            if child.is_dir() and (child / "package.xml").exists():
                packages.append(_package_info(child))

        return jsonify({"success": True, "count": len(packages), "items": packages})

    @bp.route("/ros2/packages/<package_name>", methods=["GET"])
    def ros2_package_detail(package_name: str):
        pkg_dir = _find_package_dir(package_name)
        if not pkg_dir:
            return jsonify({"success": False, "message": "Package not found"}), 404

        return jsonify({"success": True, "package": _package_info(pkg_dir)})

    @bp.route("/ros2/packages/<package_name>/launch", methods=["GET"])
    def ros2_package_launch(package_name: str):
        pkg_dir = _find_package_dir(package_name)
        if not pkg_dir:
            return jsonify({"success": False, "message": "Package not found"}), 404

        return jsonify(
            {
                "success": True,
                "package": pkg_dir.name,
                "launch_files": _package_launch_paths(pkg_dir),
            }
        )

    @bp.route("/ros2/packages/<package_name>/files", methods=["GET"])
    def ros2_package_files(package_name: str):
        pkg_dir = _find_package_dir(package_name)
        if not pkg_dir:
            return jsonify({"success": False, "message": "Package not found"}), 404

        try:
            limit = int(request.args.get("limit", 200))
        except ValueError:
            return jsonify({"success": False, "message": "Invalid limit"}), 400

        files = _package_files(pkg_dir, limit=limit)
        return jsonify(
            {
                "success": True,
                "package": pkg_dir.name,
                "count": len(files),
                "items": files,
            }
        )


__all__ = ["register_ros2_package_routes"]
