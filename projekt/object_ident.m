%y - odpowiedz skokowa
%t = [0.01:0.01:10];
%transpose(t);
%data = [y t] - macierz dwukolumnowa, zawieraj�ca warto�� odpowiedzi i czas
obj = tfest(data, 1, 0) %estymacja transmitancji obiektem inercyjnym I rz�du