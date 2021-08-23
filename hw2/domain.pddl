(define (domain bookWorld)

    (:requirements
        :equality
        :typing
        :strips
    )

    (:types
        robot
        book
        bin
        location
        subject
        size
    )


    (:predicates
        (Book_At ?a - book ?b - location)
        (Bin_At ?a - bin ?b - location)
        (Book_Subject ?a - book ?b - subject)
        (Book_Size ?a - book ?b - size)
        (Bin_Subject ?a - bin ?b - subject)
        (Bin_Size ?a - bin ?b - size)
        (Robot_At ?a - robot ?b - location)
        (Empty_Basket ?a - robot)
        (In_Basket ?b - book)
    )

    ; Pick up a book from the location.
    (:action pick
        :parameters (?book - book ?bot - robot ?locbk - location)
        :precondition (and 
            ; ADD ALL PRECONDITIONS HERE
        )
        :effect (and
            ; ADD ALL EFFECTS HERE
        )
    )

    ; Place the book on the robot into the bin.
    ; The robot must be at the drop-off location, must be holding the book, and
    ; the book subject and size must match that of the book.
    (:action place
        :parameters (?book - book ?bot - robot ?locbt - location ?bin - bin ?locbn - location ?sizebn - size ?subbn - subject)
        :precondition (and 
            ; ADD ALL PRECONDITIONS HERE
        )
        :effect (and
            ; ADD ALL EFFECTS HERE
        )
    )
    
    ; Move the robot from one location to another.
    (:action move
        :parameters (?bot - robot ?oldloc - location ?newloc - location)
        :precondition (and 
            ; ADD ALL PRECONDITIONS HERE
        )
        :effect (and
            ; ADD ALL EFFECTS HERE
        )
    )
)
