//! Actual output selection, separate from the requested algorithm parameters.

use super::runtime::Selection;
use crate::config::Algorithm;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Report {
    pub requested: bool,
    pub active: bool,
    pub configured: Algorithm,
    pub selection: Selection,
}

impl Report {
    pub fn label(self) -> (&'static str, bool) {
        match (self.active, self.requested, self.configured, self.selection) {
            (false, false, _, _) => ("EST off", false),
            (false, true, _, _) => ("EST requested; inactive", true),
            (true, false, _, _) => ("EST stopping", false),
            (true, true, Algorithm::Ekf2, Selection::Filter) => ("EST EKF2 active", false),
            (true, true, Algorithm::Ekf2, Selection::Mahony) => ("EST Mahony fallback", true),
            (true, true, Algorithm::Mahony, Selection::Filter | Selection::Mahony) => {
                ("EST Mahony active", false)
            }
            (true, true, _, Selection::Invalid) => ("EST active; no fresh attitude", true),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn requested_algorithm_is_not_mistaken_for_actual_output_or_fallback() {
        let mut report = Report {
            requested: false,
            active: false,
            configured: Algorithm::Ekf2,
            selection: Selection::Invalid,
        };
        assert_eq!(report.label(), ("EST off", false));
        report.requested = true;
        assert_eq!(report.label(), ("EST requested; inactive", true));
        report.active = true;
        assert_eq!(report.label(), ("EST active; no fresh attitude", true));
        report.selection = Selection::Filter;
        assert_eq!(report.label(), ("EST EKF2 active", false));
        report.selection = Selection::Mahony;
        assert_eq!(report.label(), ("EST Mahony fallback", true));
        report.configured = Algorithm::Mahony;
        assert_eq!(report.label(), ("EST Mahony active", false));
    }
}
