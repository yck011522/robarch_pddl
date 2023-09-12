(define (domain beam_assembly)
    (:requirements :strips)
    ; :derived-predicates
    (:predicates
        ;; There are three beam position states: AtStorage, AtRobot, and AtAssembled. One and Only One is True at the same time.
        (BeamAtStorage ?beam)
        (BeamAtAssembled ?beam)
    )
    (:action assemble_beam
        :parameters (?beam)
        :precondition (BeamAtStorage ?beam)
        :effect(and (not (BeamAtStorage ?beam)) (BeamAtAssembled ?beam))
    )
)