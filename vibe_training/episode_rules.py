"""Fixed-duration scoring rules; no simulator or learning library required."""
from dataclasses import dataclass
import math


@dataclass
class EpisodeRules:
    horizon: int = 400
    dt: float = 0.03
    steps: int = 0
    failed_at: int | None = None

    def advance(self):
        if self.done:
            raise RuntimeError('Episode termine : reset requis.')
        self.steps += 1

    def fail(self):
        if self.failed_at is None:
            self.failed_at = self.steps

    @property
    def failed(self):
        return self.failed_at is not None

    @property
    def done(self):
        return self.steps >= self.horizon

    @property
    def elapsed(self):
        return self.steps * self.dt

    @property
    def remaining(self):
        return max(0., 1. - self.steps / self.horizon)

    @property
    def survival(self):
        return (self.failed_at if self.failed else self.steps) * self.dt

    def score(self, raw_reward):
        # A failure gets the lowest possible per-step score at EVERY remaining
        # step. Early failure cannot remove future penalties from the return.
        if self.failed:
            return -1.
        if not math.isfinite(raw_reward):
            raise ValueError('Recompense non finie.')
        return max(-1., min(1., raw_reward / 10.))


def verify_rules():
    def total(raw, failed_at=None, gamma=1.):
        rules = EpisodeRules()
        result = 0.
        while not rules.done:
            rules.advance()
            if rules.steps == failed_at:
                rules.fail()
            result += gamma ** (rules.steps - 1) * rules.score(raw)
        if rules.elapsed != 12. or rules.steps != 400 or rules.remaining != 0.:
            raise AssertionError('Duree fixe incorrecte.')
        return result
    for gamma in (1., 0.99):
        for raw in (-1000., -10., -3., 0., 5., 1000.):
            early, late, alive = total(raw, 38, gamma), total(raw, 140, gamma), total(raw, None, gamma)
            if early > late + 1e-8 or late > alive + 1e-8:
                raise AssertionError('Une chute precoce ameliore encore le score.')
    rules = EpisodeRules()
    rules.advance()
    rules.fail()
    while not rules.done:
        rules.advance()
        if rules.score(1000.) != -1. or rules.survival != 0.03:
            raise AssertionError('Echec non persistant.')
    if not math.isclose(total(-3., 38), -374.1, abs_tol=1e-8):
        raise AssertionError('Score de reference incorrect.')
    return {'early_failure': total(-3., 38), 'later_failure': total(-3., 140),
            'no_failure': total(-3.), 'duration_s': 12.}


if __name__ == '__main__':
    print('Verification des regles :', verify_rules())
