"""Scorer registry. Add the method scorer here (T-009)."""

from .base import Scorer
from .reference import NullScorer, VisibleCountScorer

REGISTRY = {
    NullScorer.name: NullScorer,
    VisibleCountScorer.name: VisibleCountScorer,
    # 'fisher_bearing': FisherBearingScorer,   # T-009, after GATE-1
}


def make_scorer(name, landmarks, camera, **params) -> Scorer:
    try:
        cls = REGISTRY[name]
    except KeyError:
        raise ValueError(f'unknown scorer {name!r}; known: {sorted(REGISTRY)}') from None
    return cls(landmarks, camera, **params)


__all__ = ['Scorer', 'NullScorer', 'VisibleCountScorer', 'REGISTRY', 'make_scorer']
