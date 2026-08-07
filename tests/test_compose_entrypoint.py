import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
ENV_FILE = ROOT / ".env"


class ComposeEntrypointTest(unittest.TestCase):
    def test_env_loads_all_peanut01_compose_layers_in_order(self):
        values = {}
        for raw_line in ENV_FILE.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            values[key] = value

        self.assertEqual(
            ":".join(
                (
                    "docker-compose.yaml",
                    "docker-compose.override.yaml",
                    "docker-compose.peanut01-dry-run.yaml",
                    "docker-compose.peanut01-video.yaml",
                )
            ),
            values.get("COMPOSE_FILE"),
        )
        self.assertEqual(":", values.get("COMPOSE_PATH_SEPARATOR"))


if __name__ == "__main__":
    unittest.main()
