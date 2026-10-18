"""Scorer registry. The method scorer (ADR-001, S2*) is `marker_soft_count`."""

from .base import Scorer
from .marker_soft_count import MarkerSoftCountScorer
from .reference import NullScorer, VisibleCountScorer

REGISTRY = {
    NullScorer.name: NullScorer,
    VisibleCountScorer.name: VisibleCountScorer,
    MarkerSoftCountScorer.name: MarkerSoftCountScorer,
}


def make_scorer(name, landmarks, camera, **params) -> Scorer:
    try:
        cls = REGISTRY[name]
    except KeyError:
        raise ValueError(f'unknown scorer {name!r}; known: {sorted(REGISTRY)}') from None
    return cls(landmarks, camera, **params)


__all__ = ['Scorer', 'NullScorer', 'VisibleCountScorer', 'MarkerSoftCountScorer',
           'REGISTRY', 'make_scorer']
