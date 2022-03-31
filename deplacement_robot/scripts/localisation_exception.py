

#definition des classes erreurs
class UntrustworthyLocalisationError(Exception):
    def __init__(self, erreur):
        super(UntrustworthyLocalisationError, self).__init__("La localisation est de mauvaise facture, l'erreur est de {:.2f}".format(erreur))

class MatchingError(Exception):
    def __init__(self, message):
        self.message = message
        super(MatchingError, self).__init__()
