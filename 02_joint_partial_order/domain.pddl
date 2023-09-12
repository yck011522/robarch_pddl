(define (domain joint_partial_order)
    (:requirements :strips :negative-preconditions)
    (:predicates
        (BeamAtStorage ?beam)
        (BeamAtAssembled ?beam)
        (Joint ?earlierbeam ?laterbeam)
    )
    (:action assemble_beam
        :parameters (?beam)
        :precondition (and
            (BeamAtStorage ?beam)
            (not (exists (?earlierbeam) (and
                (Joint ?earlierbeam ?beam)
                (not(BeamAtAssembled ?earlierbeam))
            )))
        )
        :effect(and (not (BeamAtStorage ?beam)) (BeamAtAssembled ?beam))
    )
)