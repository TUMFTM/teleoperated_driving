import pathlib
import unittest

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
DOCKERFILE = ROOT / "docker/dockerfile"
ENV_FILE = ROOT / ".env"
BASE_COMPOSE = ROOT / "docker-compose.yaml"
OVERRIDE_COMPOSE = ROOT / "docker-compose.override.yaml"
DRY_RUN_COMPOSE = ROOT / "docker-compose.peanut01-dry-run.yaml"
VIDEO_COMPOSE = ROOT / "docker-compose.peanut01-video.yaml"
WORKFLOW = ROOT / ".github/workflows/publish-images.yml"


def load_yaml(testcase, path):
    testcase.assertTrue(path.exists(), f"missing {path.relative_to(ROOT)}")
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def load_env():
    values = {}
    for raw_line in ENV_FILE.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key] = value
    return values


class GhcrImageTest(unittest.TestCase):
    def test_runtime_services_use_public_ghcr_images(self):
        env = load_env()
        compose = load_yaml(self, BASE_COMPOSE)["services"]

        self.assertEqual(
            "ghcr.io/sl-kai/teleoperated-driving-operator",
            env["TOD_OPERATOR_IMAGE"],
        )
        self.assertEqual(
            "ghcr.io/sl-kai/teleoperated-driving-vehicle",
            env["TOD_VEHICLE_IMAGE"],
        )
        self.assertEqual(
            "${TOD_OPERATOR_IMAGE:?}:${DOCKER_TAG:?}",
            compose["tod_operator"]["image"],
        )
        self.assertEqual(
            "${TOD_VEHICLE_IMAGE:?}:${DOCKER_TAG:?}",
            compose["tod_vehicle"]["image"],
        )

    def test_runtime_images_contain_deployment_tools(self):
        dockerfile = DOCKERFILE.read_text(encoding="utf-8")

        for tool in (
            "set_g923_autocenter.py",
            "logitechg923.yaml",
            "peanut01_lidar_domain_bridge.py",
            "peanut01_vehicle_state_bridge.py",
            "peanut01_vehicle_state_mapping.py",
        ):
            self.assertIn(tool, dockerfile)

    def test_compose_profiles_do_not_mount_build_overlays_or_source_files(self):
        for path in (OVERRIDE_COMPOSE, DRY_RUN_COMPOSE, VIDEO_COMPOSE):
            text = path.read_text(encoding="utf-8")
            self.assertNotIn(".peanut01_", text)
            self.assertNotIn("source: ./src/", text)
            self.assertNotIn("source: ./work/", text)

    def test_workflow_publishes_both_images_for_both_architectures(self):
        workflow = WORKFLOW.read_text(encoding="utf-8")

        self.assertIn("linux/amd64,linux/arm64", workflow)
        self.assertIn("target: tod_operator", workflow)
        self.assertIn("target: tod_vehicle", workflow)
        self.assertIn("ghcr.io/sl-kai/teleoperated-driving-operator", workflow)
        self.assertIn("ghcr.io/sl-kai/teleoperated-driving-vehicle", workflow)
        self.assertIn("docker/login-action", workflow)
        self.assertIn("docker/build-push-action", workflow)


if __name__ == "__main__":
    unittest.main()
